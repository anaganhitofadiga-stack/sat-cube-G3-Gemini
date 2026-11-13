# Os ficheiros de controlo ficam na raiz do v5 (partilhados com o teste do PC)
# e sao compilados a partir daqui: uma unica copia, sem duplicacao.
idf_component_register(
    SRCS
        "main.c"
        "app_usb.c"
        "hal_cubo.c"
        "drv_moteus.c"
        # "drv_mcp2518fd.c"          # acrescentar quando o Edson o tiver
        "../../app_controlo.c"
        "../../controlo_atitude.c"
        "../../controlo_visual.c"
    INCLUDE_DIRS
        "."
        "../.."
    REQUIRES
        driver
        esp_timer
        vfs
)

# O codigo de controlo e C99 limpo; manter os avisos ligados ajuda a apanhar
# erros de unidades e de conversoes implicitas.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra)
