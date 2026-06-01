#Requires -Version 5.1
# Ball Balancer -- Firmware Tool
# Iniciado por bb_firmware_tool.bat, que carrega o perfil ESP-IDF primeiro.
# Uso: bb_firmware_tool.bat [COMx]   ex: bb_firmware_tool.bat COM11

param(
    [string]$InitPort = ''
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$ToolDir  = Split-Path -Parent $MyInvocation.MyCommand.Path
$FwRoot   = (Resolve-Path (Join-Path $ToolDir '..')).Path
$BuildDir = Join-Path $FwRoot 'build'
$LogDir   = Join-Path $ToolDir 'logs'
$CfgFile  = Join-Path $ToolDir '.bb_config'
$HeaderFile = Join-Path $FwRoot 'main\touch_screen.h'

if (-not (Test-Path $LogDir)) { New-Item -ItemType Directory $LogDir -Force | Out-Null }
$LogFile = Join-Path $LogDir 'bb_tool.log'

$script:Port = ''
$script:Baud = 115200

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

function Log([string]$m) {
    $ts = Get-Date -Format 'yyyy-MM-dd HH:mm:ss'
    Add-Content $LogFile "[$ts] $m" -ErrorAction SilentlyContinue
}

function Header([string]$title = '') {
    Clear-Host
    Write-Host ''
    Write-Host '  ================================================' -ForegroundColor Blue
    Write-Host '  Ball Balancer  --  Firmware Tool  (ESP32-S3)' -ForegroundColor White
    if ($title) { Write-Host "  $title" -ForegroundColor Cyan }
    Write-Host '  ================================================' -ForegroundColor Blue
    Write-Host ''
}

function StatusLine {
    $idfOk = [bool](Get-Command idf.py -ErrorAction SilentlyContinue)
    Write-Host "  Root   : $FwRoot" -ForegroundColor Cyan
    $portLabel = if ($script:Port) { $script:Port } else { 'auto-detect on use' }
    Write-Host "  COM    : $portLabel" -ForegroundColor Cyan
    if ($idfOk) { Write-Host '  IDF    : disponivel' -ForegroundColor Green }
    else         { Write-Host '  IDF    : NAO ENCONTRADO -- carregue o perfil ESP-IDF' -ForegroundColor Red }
    Write-Host ''
}

function Pause {
    Write-Host ''
    Write-Host '  Pressione Enter para voltar ao menu...' -ForegroundColor DarkGray
    $null = Read-Host
}

function LoadConfig {
    if (-not (Test-Path $CfgFile)) { return }
    Get-Content $CfgFile | ForEach-Object {
        if ($_ -match '^PORT=(.+)$') { $script:Port = $matches[1] }
        if ($_ -match '^BAUD=(\d+)$') { $script:Baud = [int]$matches[1] }
    }
}

function SaveConfig {
    "PORT=$($script:Port)`nBAUD=$($script:Baud)" |
        Set-Content $CfgFile -Encoding ASCII -ErrorAction SilentlyContinue
}

function GetEspPort {
    try {
        Get-CimInstance Win32_PnPEntity -ErrorAction Stop |
            Where-Object { $_.Name -match 'COM\d+' -and $_.Name -match 'CP210|CH340|FTDI|Silicon Labs|USB-SERIAL' } |
            ForEach-Object { if ($_.Name -match '(COM\d+)') { $matches[1] } } |
            Select-Object -First 1
    } catch { $null }
}

function NormalizePort([string]$p) {
    if ($p -match '^\d+$') { return "COM$p" }
    return $p
}

function Test-PathSafe {
    if ($FwRoot -match '[^\x00-\x7F]') {
        Write-Host '  +------------------------------------------------------+' -ForegroundColor Red
        Write-Host '  |  ATENCAO: caminho do projeto com caractere especial! |' -ForegroundColor Red
        Write-Host '  |  O ESP-IDF nao suporta acentos ou simbolos no path.  |' -ForegroundColor Red
        Write-Host '  |  O build vai falhar com erro de kconfig.             |' -ForegroundColor Red
        Write-Host '  +------------------------------------------------------+' -ForegroundColor Red
        Write-Host ''
        Write-Host "  Caminho atual : $FwRoot" -ForegroundColor Yellow
        Write-Host '  Solucao       : mova o projeto para um caminho sem acentos.' -ForegroundColor Yellow
        Write-Host '  Sugestao      : C:\projetos\ball_balancer\' -ForegroundColor Cyan
        Write-Host ''
        Write-Host '  Pressione Enter para continuar mesmo assim...' -ForegroundColor DarkGray
        $null = Read-Host
    }
}

function Ensure-Target {
    $sdkPath = Join-Path $FwRoot 'sdkconfig'
    if (Test-Path $sdkPath) {
        $sdkContent = Get-Content $sdkPath -Raw
        if ($sdkContent -match 'CONFIG_IDF_TARGET_ESP32S3=y') { return $true }
    }
    Write-Host '  Target nao definido para esp32s3. Configurando...' -ForegroundColor Yellow
    Write-Host '  Executando: idf.py set-target esp32s3' -ForegroundColor DarkGray
    Write-Host ''
    RunIdf set-target esp32s3
    if ($LASTEXITCODE -ne 0) {
        Write-Host '  Falhou ao definir target esp32s3.' -ForegroundColor Red
        return $false
    }
    Write-Host ''
    Write-Host '  Target esp32s3 configurado.' -ForegroundColor Green
    Write-Host ''
    return $true
}

function SelectPort {
    $auto = GetEspPort
    if ($auto) {
        Write-Host "  Auto-detectado: $auto" -ForegroundColor Green
        $ans = (Read-Host "  Usar $auto? (Enter = sim  /  digite COMx para outro)").Trim()
        $script:Port = if ($ans -eq '') { $auto } else { NormalizePort $ans }
    } elseif ($script:Port) {
        Write-Host "  Ultima porta usada: $($script:Port)" -ForegroundColor Yellow
        $ans = (Read-Host "  Usar $($script:Port)? (Enter = sim  /  digite COMx para outro)").Trim()
        if ($ans -ne '') { $script:Port = NormalizePort $ans }
    } else {
        Write-Host '  Nenhum ESP32 detectado na USB.' -ForegroundColor Yellow
        $raw = (Read-Host '  Digite a porta COM (ex: COM5)').Trim()
        $script:Port = NormalizePort $raw
    }
    SaveConfig
    return ($script:Port -ne '')
}

function RunIdf {
    Push-Location $FwRoot
    try {
        & idf.py @args
        return $LASTEXITCODE
    } finally {
        Pop-Location
    }
}

function ShowResult([int]$ec, [datetime]$start) {
    $dur = [int]((Get-Date) - $start).TotalSeconds
    Write-Host ''
    Write-Host '  ------------------------------------------------' -ForegroundColor DarkGray
    if ($ec -eq 0) {
        Write-Host "  [OK]   Concluido em ${dur}s" -ForegroundColor Green
    } else {
        Write-Host "  [FAIL] Falhou (exit $ec) em ${dur}s" -ForegroundColor Red
    }
    Log "ec=$ec dur=${dur}s"
}

# ---------------------------------------------------------------------------
# Acoes de build
# ---------------------------------------------------------------------------

function Do-Build {
    Header 'Build'
    if (-not (Get-Command idf.py -ErrorAction SilentlyContinue)) {
        Write-Host '  idf.py nao encontrado. Abra pelo atalho ESP-IDF ou execute o .bat.' -ForegroundColor Red
        Pause; return
    }
    if (-not (Ensure-Target)) { Pause; return }
    Write-Host "  Projeto : $FwRoot" -ForegroundColor Cyan
    Write-Host '  Comando : idf.py build' -ForegroundColor DarkGray
    Write-Host ''
    Log 'build start'
    $t = Get-Date
    RunIdf build
    ShowResult $LASTEXITCODE $t
    Pause
}

function Do-FullClean {
    Header 'Full Clean + Rebuild'
    Write-Host '  Remove a pasta build/ e reconstroi tudo do zero.' -ForegroundColor Yellow
    Write-Host '  O primeiro build demora varios minutos.' -ForegroundColor DarkGray
    Write-Host ''
    $ans = Read-Host '  Digite YES para confirmar'
    if ($ans -cne 'YES') { Write-Host '  Cancelado.'; Pause; return }
    Write-Host ''
    Log 'fullclean+build start'
    $t = Get-Date
    RunIdf fullclean
    if ($LASTEXITCODE -eq 0) {
        Write-Host ''
        Write-Host '  -- Configurando target esp32s3 --' -ForegroundColor Cyan
        Write-Host ''
        RunIdf set-target esp32s3
    }
    if ($LASTEXITCODE -eq 0) {
        Write-Host ''
        Write-Host '  -- Iniciando build --' -ForegroundColor Cyan
        Write-Host ''
        RunIdf build
    }
    ShowResult $LASTEXITCODE $t
    Pause
}

# ---------------------------------------------------------------------------
# Acoes de flash / monitor
# ---------------------------------------------------------------------------

function Do-Flash {
    Header 'Flash'
    if (-not (SelectPort)) { Write-Host '  Porta nao definida. Cancelado.' -ForegroundColor Red; Pause; return }
    Write-Host "  Porta   : $($script:Port)" -ForegroundColor Cyan
    Write-Host '  Comando : idf.py flash' -ForegroundColor DarkGray
    Write-Host ''
    Log "flash $($script:Port)"
    $t = Get-Date
    RunIdf -p $script:Port flash
    ShowResult $LASTEXITCODE $t
    Pause
}

function Do-Monitor {
    Header 'Monitor'
    if (-not (SelectPort)) { Write-Host '  Porta nao definida. Cancelado.' -ForegroundColor Red; Pause; return }
    Write-Host "  Porta   : $($script:Port)  Baud: $($script:Baud)" -ForegroundColor Cyan
    Write-Host '  Pressione Ctrl+] para sair do monitor.' -ForegroundColor DarkGray
    Write-Host ''
    Log "monitor $($script:Port)"
    Push-Location $FwRoot
    try { & idf.py -p $script:Port -b $script:Baud monitor }
    finally { Pop-Location }
    Pause
}

function Do-FlashMonitor {
    Header 'Flash + Monitor'
    if (-not (SelectPort)) { Write-Host '  Porta nao definida. Cancelado.' -ForegroundColor Red; Pause; return }
    Write-Host "  Porta   : $($script:Port)  Baud: $($script:Baud)" -ForegroundColor Cyan
    Write-Host '  Grava firmware e abre monitor. Ctrl+] para sair.' -ForegroundColor DarkGray
    Write-Host ''
    Log "flash+monitor $($script:Port)"
    Push-Location $FwRoot
    try { & idf.py -p $script:Port -b $script:Baud flash monitor }
    finally { Pop-Location }
    Pause
}

# ---------------------------------------------------------------------------
# Acoes de ferramentas
# ---------------------------------------------------------------------------

function Do-Detect {
    Header 'Detectar dispositivos USB'
    Write-Host '  Escaneando...' -ForegroundColor DarkGray
    Write-Host ''
    try {
        $all  = @(Get-CimInstance Win32_PnPEntity -ErrorAction Stop | Where-Object { $_.Name -match 'COM\d+' })
        $esp  = @($all | Where-Object { $_.Name -match 'CP210|CH340|FTDI|Silicon Labs|USB-SERIAL' })
        $rest = @($all | Where-Object { $_.Name -notmatch 'CP210|CH340|FTDI|Silicon Labs|USB-SERIAL' })
    } catch { $all = @(); $esp = @(); $rest = @() }

    Write-Host '  Adaptadores compativeis com ESP32:' -ForegroundColor Cyan
    if ($esp.Count -gt 0) {
        $esp | ForEach-Object { Write-Host "    $($_.Name)" -ForegroundColor White }
    } else {
        Write-Host '    (nenhum detectado -- conecte o dispositivo ou instale o driver)' -ForegroundColor Yellow
    }
    Write-Host ''
    Write-Host '  Outras portas COM:' -ForegroundColor DarkGray
    if ($rest.Count -gt 0) {
        $rest | ForEach-Object { Write-Host "    $($_.Name)" -ForegroundColor DarkGray }
    } else {
        Write-Host '    (nenhuma)' -ForegroundColor DarkGray
    }
    Write-Host ''
    Write-Host '  Dica: CP210x = Silicon Labs   CH340 = modulo comum   FTDI = FT232' -ForegroundColor DarkGray
    Pause
}

function Do-Diagnostics {
    Header 'Diagnosticos'
    $p = 0; $w = 0; $f = 0

    function Pass([string]$lbl, [string]$note = '') {
        Write-Host ("  PASS  {0,-32} {1}" -f $lbl, $note) -ForegroundColor Green;  $script:p++
    }
    function Warn([string]$lbl, [string]$note = '') {
        Write-Host ("  WARN  {0,-32} {1}" -f $lbl, $note) -ForegroundColor Yellow; $script:w++
    }
    function Fail([string]$lbl, [string]$note = '') {
        Write-Host ("  FAIL  {0,-32} {1}" -f $lbl, $note) -ForegroundColor Red;    $script:f++
    }

    # Caminho do projeto
    if ($FwRoot -match '[^\x00-\x7F]') {
        Fail 'Caminho do projeto' 'contem caracteres especiais -- build vai falhar'
    } else {
        Pass 'Caminho do projeto' 'sem caracteres especiais'
    }

    # Target ESP32-S3
    $sdkPath = Join-Path $FwRoot 'sdkconfig'
    if (Test-Path $sdkPath) {
        $sdkContent = Get-Content $sdkPath -Raw
        if ($sdkContent -match 'CONFIG_IDF_TARGET_ESP32S3=y') {
            Pass 'IDF Target' 'esp32s3'
        } else {
            $tgt = if ($sdkContent -match 'CONFIG_IDF_TARGET="([^"]+)"') { $matches[1] } else { 'desconhecido' }
            Fail 'IDF Target' "configurado para '$tgt' -- execute set-target esp32s3"
        }
    } else {
        Warn 'IDF Target' 'sdkconfig ausente -- sera definido no proximo build'
    }

    # idf.py
    if (Get-Command idf.py -ErrorAction SilentlyContinue) { Pass 'idf.py' }
    else { Fail 'idf.py' 'nao encontrado -- carregue o perfil ESP-IDF' }

    # Arquivos do projeto
    if (Test-Path (Join-Path $FwRoot 'CMakeLists.txt'))        { Pass 'CMakeLists.txt' }
    else { Fail 'CMakeLists.txt' 'raiz do projeto incorreta?' }

    if (Test-Path (Join-Path $FwRoot 'main\touch_screen.h'))   { Pass 'touch_screen.h' }
    else { Fail 'touch_screen.h' 'arquivo nao encontrado em main/' }

    if (Test-Path (Join-Path $FwRoot 'main\touch_screen.c'))   { Pass 'touch_screen.c' }
    else { Fail 'touch_screen.c' 'arquivo nao encontrado em main/' }

    if (Test-Path (Join-Path $FwRoot 'main\main.c'))           { Pass 'main.c' }
    else { Fail 'main.c' 'arquivo nao encontrado em main/' }

    $sdk = Join-Path $FwRoot 'sdkconfig'
    $def = Join-Path $FwRoot 'sdkconfig.defaults'
    if     (Test-Path $sdk) { Pass 'sdkconfig' }
    elseif (Test-Path $def) { Warn 'sdkconfig' 'ausente -- sera criado no primeiro build' }
    else                    { Warn 'sdkconfig' 'nenhum sdkconfig ou sdkconfig.defaults' }

    # Binario compilado
    $bin = Get-ChildItem $BuildDir -Filter 'ball_balancer.bin' -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($bin) { Pass 'Binario compilado' ("{0} KB" -f [int]($bin.Length / 1KB)) }
    else       { Warn 'Binario compilado' 'execute Build primeiro' }

    # USB
    $port = GetEspPort
    if ($port) { Pass 'ESP32 na USB' $port }
    else        { Warn 'ESP32 na USB' 'nenhum dispositivo detectado' }

    # IDF_PATH
    if ($env:IDF_PATH) { Pass 'IDF_PATH' $env:IDF_PATH }
    else                { Warn 'IDF_PATH' 'nao definido nesta sessao' }

    # Python
    $pyOk = [bool](Get-Command python -ErrorAction SilentlyContinue)
    if ($pyOk) {
        $pyVer = & python --version 2>&1
        Pass 'Python' $pyVer
    } else { Warn 'Python' 'nao encontrado -- necessario para o visualizador' }

    # pygame + pyserial
    if ($pyOk) {
        $pkgOk = & python -c "import pygame, serial" 2>&1
        if ($LASTEXITCODE -eq 0) { Pass 'pygame + pyserial' }
        else { Warn 'pygame + pyserial' "execute: pip install -r tools\requirements.txt" }
    }

    # Visualizador
    if (Test-Path (Join-Path $ToolDir 'visualizer.py')) { Pass 'visualizer.py' }
    else { Warn 'visualizer.py' 'nao encontrado em tools/' }

    # Disco
    try {
        $free = [math]::Round((Get-PSDrive C -ErrorAction Stop).Free / 1GB, 1)
        if ($free -lt 1) { Warn 'Disco (C:)' "$free GB livres -- build pode falhar" }
        else              { Pass 'Disco (C:)' "$free GB livres" }
    } catch { Warn 'Disco (C:)' 'nao foi possivel verificar' }

    Write-Host ''
    Write-Host "  PASS: $($script:p)  WARN: $($script:w)  FAIL: $($script:f)" -ForegroundColor White
    Pause
}

# ---------------------------------------------------------------------------
# Definir porta COM
# ---------------------------------------------------------------------------

function Do-SetPort {
    Header 'Definir Porta COM'
    $auto = GetEspPort
    if ($auto) {
        Write-Host "  ESP32 detectado automaticamente: $auto" -ForegroundColor Green
    }
    if ($script:Port) {
        Write-Host "  Porta atual: $($script:Port)" -ForegroundColor Cyan
    } else {
        Write-Host "  Porta atual: (nao definida)" -ForegroundColor Yellow
    }
    Write-Host ''
    $raw = (Read-Host '  Digite a porta COM (ex: COM5 ou 5, Enter para manter atual)').Trim()
    if ($raw -ne '') {
        $script:Port = NormalizePort $raw
        SaveConfig
        Write-Host "  Porta definida: $($script:Port)" -ForegroundColor Green
        Log "porta definida manualmente: $($script:Port)"
    } else {
        Write-Host '  Porta nao alterada.' -ForegroundColor DarkGray
    }
    Pause
}

# ---------------------------------------------------------------------------
# Calibracao da tela resistiva
# ---------------------------------------------------------------------------

function Do-Calibration {
    Header 'Calibracao da Tela Resistiva'

    if (-not (Test-Path $HeaderFile)) {
        Write-Host "  Arquivo nao encontrado: $HeaderFile" -ForegroundColor Red
        Pause; return
    }

    $content = Get-Content $HeaderFile -Raw

    # Le valores atuais
    $curXMin = if ($content -match '#define\s+TOUCH_X_RAW_MIN\s+(\d+)') { $matches[1] } else { '?' }
    $curXMax = if ($content -match '#define\s+TOUCH_X_RAW_MAX\s+(\d+)') { $matches[1] } else { '?' }
    $curYMin = if ($content -match '#define\s+TOUCH_Y_RAW_MIN\s+(\d+)') { $matches[1] } else { '?' }
    $curYMax = if ($content -match '#define\s+TOUCH_Y_RAW_MAX\s+(\d+)') { $matches[1] } else { '?' }

    Write-Host '  Valores atuais em touch_screen.h:' -ForegroundColor Cyan
    Write-Host ''
    Write-Host ("  {0,-22} {1,6}" -f 'TOUCH_X_RAW_MIN', $curXMin) -ForegroundColor White
    Write-Host ("  {0,-22} {1,6}" -f 'TOUCH_X_RAW_MAX', $curXMax) -ForegroundColor White
    Write-Host ("  {0,-22} {1,6}" -f 'TOUCH_Y_RAW_MIN', $curYMin) -ForegroundColor White
    Write-Host ("  {0,-22} {1,6}" -f 'TOUCH_Y_RAW_MAX', $curYMax) -ForegroundColor White
    Write-Host ''
    Write-Host '  Como calibrar: flash + abrir Monitor, mover a esfera ate cada canto' -ForegroundColor DarkGray
    Write-Host '  e anotar os valores RAW exibidos. Cole-os abaixo.' -ForegroundColor DarkGray
    Write-Host '  Deixe em branco para manter o valor atual.' -ForegroundColor DarkGray
    Write-Host ''

    function ReadInt([string]$prompt, [string]$current) {
        $r = (Read-Host "  $prompt (atual: $current)").Trim()
        if ($r -eq '') { return $null }
        if ($r -match '^\d+$') { return [int]$r }
        Write-Host "  Valor invalido ignorado." -ForegroundColor Yellow
        return $null
    }

    $newXMin = ReadInt 'TOUCH_X_RAW_MIN' $curXMin
    $newXMax = ReadInt 'TOUCH_X_RAW_MAX' $curXMax
    $newYMin = ReadInt 'TOUCH_Y_RAW_MIN' $curYMin
    $newYMax = ReadInt 'TOUCH_Y_RAW_MAX' $curYMax

    $changed = $false

    if ($null -ne $newXMin) {
        $content = $content -replace '(#define\s+TOUCH_X_RAW_MIN\s+)\d+', "`${1}$newXMin"
        Write-Host "  TOUCH_X_RAW_MIN -> $newXMin" -ForegroundColor Green
        $changed = $true
    }
    if ($null -ne $newXMax) {
        $content = $content -replace '(#define\s+TOUCH_X_RAW_MAX\s+)\d+', "`${1}$newXMax"
        Write-Host "  TOUCH_X_RAW_MAX -> $newXMax" -ForegroundColor Green
        $changed = $true
    }
    if ($null -ne $newYMin) {
        $content = $content -replace '(#define\s+TOUCH_Y_RAW_MIN\s+)\d+', "`${1}$newYMin"
        Write-Host "  TOUCH_Y_RAW_MIN -> $newYMin" -ForegroundColor Green
        $changed = $true
    }
    if ($null -ne $newYMax) {
        $content = $content -replace '(#define\s+TOUCH_Y_RAW_MAX\s+)\d+', "`${1}$newYMax"
        Write-Host "  TOUCH_Y_RAW_MAX -> $newYMax" -ForegroundColor Green
        $changed = $true
    }

    if (-not $changed) {
        Write-Host '  Nenhuma alteracao.' -ForegroundColor Yellow
        Pause; return
    }

    [System.IO.File]::WriteAllText($HeaderFile, $content, [System.Text.Encoding]::UTF8)
    Write-Host ''
    Write-Host '  Valores salvos em touch_screen.h.' -ForegroundColor Green
    Write-Host '  Execute Build (1) e Flash (3) para gravar no ESP32.' -ForegroundColor Cyan
    Log "calibracao salva X:$newXMin-$newXMax Y:$newYMin-$newYMax"
    Pause
}

# ---------------------------------------------------------------------------
# Visualizador Python
# ---------------------------------------------------------------------------

function Do-PIDTuner {
    Header 'Ajuste PID em Tempo Real'

    $script = Join-Path $ToolDir 'pid_tuner.py'
    if (-not (Test-Path $script)) {
        Write-Host '  pid_tuner.py nao encontrado em tools/.' -ForegroundColor Red
        Pause; return
    }

    if (-not (Get-Command python -ErrorAction SilentlyContinue)) {
        Write-Host '  Python nao encontrado no PATH.' -ForegroundColor Red
        Pause; return
    }

    $pkgCheck = & python -c "import pygame, serial" 2>&1
    if ($LASTEXITCODE -ne 0) {
        Write-Host '  pygame ou pyserial nao instalados.' -ForegroundColor Yellow
        $ans = (Read-Host '  Instalar agora? (Enter = sim / N = nao)').Trim()
        if ($ans -ne 'N' -and $ans -ne 'n') {
            & pip install -r (Join-Path $ToolDir 'requirements.txt')
            if ($LASTEXITCODE -ne 0) { Write-Host '  Instalacao falhou.' -ForegroundColor Red; Pause; return }
        } else { Pause; return }
    }

    if (-not (SelectPort)) { Write-Host '  Porta nao definida. Cancelado.' -ForegroundColor Red; Pause; return }

    Write-Host ''
    Write-Host '  Requer firmware em modo CONTROL (padrao).' -ForegroundColor Cyan
    Write-Host ''
    Write-Host '  Teclas:' -ForegroundColor White
    Write-Host '   Q/A = KP sobe/desce     W/S = KI sobe/desce     E/D = KD sobe/desce' -ForegroundColor DarkGray
    Write-Host '   Tab = muda passo (1%/5%/10%/25%)   Espaco = SALVAR NO NVS' -ForegroundColor DarkGray
    Write-Host ''
    Write-Host "  Iniciando PID Tuner -- porta $($script:Port)" -ForegroundColor Green

    Log "pid_tuner $($script:Port)"
    Start-Process python -ArgumentList "`"$script`" $($script:Port) $($script:Baud)" `
                         -WorkingDirectory $ToolDir
    Pause
}

function Do-Visualizer {
    Header 'Visualizador em Tempo Real'

    $vizScript = Join-Path $ToolDir 'visualizer.py'
    if (-not (Test-Path $vizScript)) {
        Write-Host '  visualizer.py nao encontrado em tools/.' -ForegroundColor Red
        Pause; return
    }

    if (-not (Get-Command python -ErrorAction SilentlyContinue)) {
        Write-Host '  Python nao encontrado no PATH.' -ForegroundColor Red
        Write-Host '  Instale em https://python.org e adicione ao PATH.' -ForegroundColor DarkGray
        Pause; return
    }

    # Verifica dependencias
    $pkgCheck = & python -c "import pygame, serial" 2>&1
    if ($LASTEXITCODE -ne 0) {
        Write-Host '  pygame ou pyserial nao instalados.' -ForegroundColor Yellow
        $ans = (Read-Host '  Instalar agora? (Enter = sim / N = nao)').Trim()
        if ($ans -ne 'N' -and $ans -ne 'n') {
            Write-Host ''
            & pip install -r (Join-Path $ToolDir 'requirements.txt')
            if ($LASTEXITCODE -ne 0) {
                Write-Host '  Instalacao falhou. Verifique sua conexao.' -ForegroundColor Red
                Pause; return
            }
        } else {
            Pause; return
        }
    }

    # Seleciona porta
    if (-not (SelectPort)) { Write-Host '  Porta nao definida. Cancelado.' -ForegroundColor Red; Pause; return }

    Write-Host ''
    Write-Host "  Iniciando visualizador -- porta $($script:Port)" -ForegroundColor Cyan
    Write-Host '  O visualizador abre em uma janela separada.' -ForegroundColor DarkGray
    Write-Host '  Mantenha o Monitor serial FECHADO (conflito de porta).' -ForegroundColor Yellow
    Write-Host ''

    Log "visualizer $($script:Port)"
    Start-Process python -ArgumentList "`"$vizScript`" $($script:Port) $($script:Baud)" `
                         -WorkingDirectory $ToolDir
    Write-Host '  Visualizador iniciado.' -ForegroundColor Green
    Pause
}

# ---------------------------------------------------------------------------
# Menuconfig e erase
# ---------------------------------------------------------------------------

function Do-Menuconfig {
    Header 'Menuconfig'
    Write-Host '  Abrindo menu de configuracoes do firmware...' -ForegroundColor Cyan
    Write-Host '  Salvar: S    Sair: Q' -ForegroundColor DarkGray
    Write-Host ''
    Log 'menuconfig'
    RunIdf menuconfig | Out-Null
    Pause
}

function Do-Erase {
    Header 'Apagar Flash  -- PERIGO'
    if (-not (SelectPort)) { Write-Host '  Porta nao definida. Cancelado.' -ForegroundColor Red; Pause; return }
    Write-Host ''
    Write-Host '  +--------------------------------------------------+' -ForegroundColor Red
    Write-Host '  |  ATENCAO: APAGA TODA A FLASH (firmware + NVS)   |' -ForegroundColor Red
    Write-Host "  |  Porta alvo: $($script:Port.PadRight(36))|" -ForegroundColor Red
    Write-Host '  +--------------------------------------------------+' -ForegroundColor Red
    Write-Host ''
    $ans = Read-Host '  Digite YES para confirmar'
    if ($ans -cne 'YES') { Write-Host '  Cancelado.'; Pause; return }
    Log "erase $($script:Port)"
    $t = Get-Date
    RunIdf -p $script:Port erase-flash
    ShowResult $LASTEXITCODE $t
    Pause
}

# ---------------------------------------------------------------------------
# Loop principal
# ---------------------------------------------------------------------------

Test-PathSafe
LoadConfig

# Porta passada pela linha de comando tem prioridade sobre o .bb_config
if ($InitPort -ne '' -and $InitPort -ne "''") {
    $script:Port = NormalizePort $InitPort
    SaveConfig
    Log "Porta definida por argumento: $($script:Port)"
}

Log '=== Ferramenta iniciada ==='

$running = $true
while ($running) {
    Header
    StatusLine

    Write-Host '  BUILD' -ForegroundColor White
    Write-Host '   1  Build'
    Write-Host '   2  Full clean + rebuild'
    Write-Host ''
    Write-Host '  FLASH / MONITOR' -ForegroundColor White
    Write-Host '   3  Flash'
    Write-Host '   4  Monitor  (saida serial)'
    Write-Host '   5  Flash + Monitor'
    Write-Host ''
    Write-Host '  FERRAMENTAS' -ForegroundColor White
    Write-Host '   6  Detectar ESP32 na USB'
    Write-Host '   7  Diagnosticos'
    Write-Host '   8  Calibracao da tela resistiva (4 cantos, salva no NVS)' -ForegroundColor Cyan
    Write-Host '   9  Visualizador Python'
    Write-Host '  10  Menuconfig  (configuracoes do firmware)'
    Write-Host '  11  Apagar flash  (PERIGO)' -ForegroundColor Yellow
    Write-Host '  12  Definir porta COM' -ForegroundColor Cyan
    Write-Host '  13  Ajuste PID em tempo real' -ForegroundColor Green
    Write-Host ''
    Write-Host '   0  Sair' -ForegroundColor Red
    Write-Host ''

    $opt = (Read-Host '  Selecione').Trim()

    try {
        switch ($opt) {
            '1'  { Do-Build }
            '2'  { Do-FullClean }
            '3'  { Do-Flash }
            '4'  { Do-Monitor }
            '5'  { Do-FlashMonitor }
            '6'  { Do-Detect }
            '7'  { Do-Diagnostics }
            '8'  { Do-Calibration }
            '9'  { Do-Visualizer }
            '10' { Do-Menuconfig }
            '11' { Do-Erase }
            '12' { Do-SetPort }
            '13' { Do-PIDTuner }
            '0'  { $running = $false }
            ''   { }
            default {
                Write-Host "  Opcao desconhecida: $opt" -ForegroundColor Yellow
                Start-Sleep -Milliseconds 600
            }
        }
    } catch {
        Write-Host "  Erro: $($_.Exception.Message)" -ForegroundColor Red
        Log "ERRO: $($_.Exception.Message)"
        Pause
    }
}

Log '=== Ferramenta encerrada ==='
Write-Host '  Ate logo.' -ForegroundColor Cyan
Write-Host ''
