# Lista as portas COM encontradas (para o ecra) e devolve no stdout a do XIAO.
# Usado pelo ligar_cubo.bat; tambem se pode correr a mao:
#   powershell -ExecutionPolicy Bypass -File ferramentas\detetar_porta.ps1
$portas = @(Get-CimInstance Win32_PnPEntity | Where-Object { $_.Name -match '\(COM\d+\)' })
if ($portas.Count -eq 0) {
    [Console]::Error.WriteLine("  (nenhuma porta COM encontrada)")
    exit 1
}
foreach ($p in $portas) { [Console]::Error.WriteLine("  " + $p.Name) }
# O XIAO aparece como USB Serial Device ou USB JTAG/serial debug unit.
$xiao = $portas | Where-Object { $_.Name -match 'USB Serial|JTAG' } | Select-Object -First 1
if (-not $xiao) { $xiao = $portas[0] }
[Console]::Out.WriteLine([regex]::Match($xiao.Name, 'COM\d+').Value)
