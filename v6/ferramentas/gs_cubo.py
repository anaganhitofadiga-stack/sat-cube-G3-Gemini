#!/usr/bin/env python3
"""
gs_cubo.py — registo e comando do cubo V5 pelo cabo USB.

Instalar:  pip install pyserial matplotlib

Registar um ensaio (Ctrl+C para terminar):
    python gs_cubo.py --porta COM7 --saida ensaio1.csv

Enquanto corre, escrever comandos e Enter (os mesmos da consola do firmware):
    a          armar
    m 1        modo atitude
    s 45       degrau de 45 graus a partir do yaw atual
    d          desarmar
    x          paragem de emergencia
    ?          ajuda

Desenhar os graficos de um ensaio ja registado:
    python gs_cubo.py --grafico ensaio1.csv
"""

import argparse
import csv
import sys
import threading
import time

COLUNAS = [
    "t_ms", "modo", "armado", "vis", "falhas",
    "roll_deg", "pitch_deg", "yaw_deg", "alvo_deg", "erro_deg",
    "wx_dps", "wy_dps", "wz_dps", "vel_des_dps", "torque_Nm",
    "roda_rpm", "dt_ms", "lim", "img_ok", "img_rej",
]

FALHAS = {
    1: "IMU", 2: "MOTEUS", 4: "SOBREVELOCIDADE",
    8: "CICLO_ATRASADO", 16: "CALIBRACAO", 32: "EMERGENCIA",
}


def nomes_falhas(mascara):
    mascara = int(mascara)
    if mascara == 0:
        return "-"
    return "+".join(nome for bit, nome in FALHAS.items() if mascara & bit)


def registar(porta, saida, mostrar):
    import serial

    # O XIAO usa USB CDC: o baudrate e ignorado, mas o pyserial exige um valor.
    ligacao = serial.Serial(porta, 115200, timeout=0.2)
    print(f"# ligado a {porta}; a escrever em {saida}")

    ficheiro = open(saida, "w", newline="", encoding="utf-8")
    escritor = csv.writer(ficheiro)
    escritor.writerow(COLUNAS)

    parar = threading.Event()
    falhas_vistas = set()
    contador = [0]

    def ler():
        while not parar.is_set():
            try:
                linha = ligacao.readline().decode("utf-8", "replace").strip()
            except Exception as erro:          # cabo desligado
                print(f"# ligacao perdida: {erro}")
                parar.set()
                return
            if not linha:
                continue
            if not linha.startswith("T,"):
                print(linha)                   # logs do ESP_LOG e respostas
                continue
            campos = linha[2:].split(",")
            if len(campos) != len(COLUNAS):
                continue
            escritor.writerow(campos)
            contador[0] += 1
            if contador[0] % 20 == 0:
                ficheiro.flush()
            mascara = int(float(campos[4]))
            if mascara and mascara not in falhas_vistas:
                falhas_vistas.add(mascara)
                print(f"# FALHA {mascara} ({nomes_falhas(mascara)})")
            if mostrar and contador[0] % 10 == 0:
                print(f"  yaw {float(campos[7]):8.2f}  alvo {float(campos[8]):8.2f}"
                      f"  erro {float(campos[9]):7.2f}  binario {float(campos[14]):8.4f}"
                      f"  roda {float(campos[15]):8.1f} rpm")

    leitor = threading.Thread(target=ler, daemon=True)
    leitor.start()

    try:
        for comando in sys.stdin:
            if parar.is_set():
                break
            ligacao.write((comando.strip() + "\n").encode("utf-8"))
    except KeyboardInterrupt:
        pass
    finally:
        parar.set()
        # Rede de seguranca: desarmar o cubo antes de sair.
        try:
            ligacao.write(b"d\n")
            time.sleep(0.2)
        except Exception:
            pass
        ligacao.close()
        ficheiro.close()
        print(f"\n# {contador[0]} amostras em {saida}")


def desenhar(ficheiro):
    import matplotlib.pyplot as plt

    dados = {nome: [] for nome in COLUNAS}
    with open(ficheiro, newline="", encoding="utf-8") as f:
        for linha in csv.DictReader(f):
            for nome in COLUNAS:
                dados[nome].append(float(linha[nome]))
    if not dados["t_ms"]:
        print("ficheiro sem amostras")
        return

    t0 = dados["t_ms"][0]
    t = [(v - t0) / 1000.0 for v in dados["t_ms"]]

    fig, eixos = plt.subplots(4, 1, sharex=True, figsize=(10, 9))
    eixos[0].plot(t, dados["yaw_deg"], label="yaw")
    eixos[0].plot(t, dados["alvo_deg"], "--", label="alvo")
    eixos[0].set_ylabel("yaw [graus]")
    eixos[0].legend(loc="best")

    eixos[1].plot(t, dados["erro_deg"], color="tab:red")
    eixos[1].axhline(1.0, color="gray", lw=0.6, ls=":")
    eixos[1].axhline(-1.0, color="gray", lw=0.6, ls=":")
    eixos[1].set_ylabel("erro [graus]")

    eixos[2].plot(t, dados["wz_dps"], label="wz medido")
    eixos[2].plot(t, dados["vel_des_dps"], "--", label="wz desejado")
    eixos[2].set_ylabel("vel. corpo [graus/s]")
    eixos[2].legend(loc="best")

    eixos[3].plot(t, dados["torque_Nm"], label="binario [N.m]")
    gemeo = eixos[3].twinx()
    gemeo.plot(t, dados["roda_rpm"], color="tab:green", label="roda [rpm]")
    gemeo.set_ylabel("roda [rpm]")
    eixos[3].set_ylabel("binario [N.m]")
    eixos[3].set_xlabel("tempo [s]")

    for eixo in eixos:
        eixo.grid(alpha=0.3)
    fig.suptitle(ficheiro)
    fig.tight_layout()
    plt.show()


def principal():
    analisador = argparse.ArgumentParser(description=__doc__)
    analisador.add_argument("--porta", help="porta serie, ex. COM7 ou /dev/ttyACM0")
    analisador.add_argument("--saida", default="ensaio.csv", help="ficheiro CSV")
    analisador.add_argument("--grafico", help="desenhar um CSV ja registado")
    analisador.add_argument("--silencioso", action="store_true",
                            help="nao mostrar amostras no ecra")
    args = analisador.parse_args()

    if args.grafico:
        desenhar(args.grafico)
    elif args.porta:
        registar(args.porta, args.saida, not args.silencioso)
    else:
        analisador.print_help()


if __name__ == "__main__":
    principal()
