@echo off
:: Ball Balancer Studio — central de controle unificada.
::
:: A GUI roda no python do sistema (que tem pygame/pyserial). NAO carregamos o
:: perfil ESP-IDF aqui de proposito: ele ativa o venv do IDF, que nao tem pygame.
:: Em vez disso, cada Build/Flash abre internamente um PowerShell proprio que
:: carrega o perfil e chama idf.py (ver IdfRunner em bb_studio.py).
::
:: Uso opcional: bb_studio.bat COM5   (sobrescreve a porta de config.py)
python "%~dp0bb_studio.py" %*
