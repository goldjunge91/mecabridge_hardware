# #!/usr/bin/env bash
# # ros2-env-setup.sh — Interaktives Setup für ROS 2 Umgebungsvariablen
# # Jetzt inklusive Selbst-Installation, damit es aus *jedem* Ordner aufrufbar ist.
# #
# # Features
# # - Fragt gewünschte Variablen ab (ROS_DISTRO, ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY)
# # - Konfiguriert nur bash (~/.bashrc) oder zusätzlich zsh (~/.zshrc)
# # - Idempotenter, markierter Block in den RC-Dateien (mit Backups)
# # - Optional: --install installiert das Skript nach ~/.local/bin, damit es überall per Name startbar ist
# # - Optional: --no-color deaktiviert farbige Ausgabe
# #
# # Nutzung:
# #   chmod +x ros2-env-setup.sh
# #   ./ros2-env-setup.sh                # interaktiver Modus
# #   ./ros2-env-setup.sh --install      # in ~/.local/bin/setup-ros2-env installieren
# #   setup-ros2-env                      # danach aus *jedem* Ordner starten (neue Shell oder RC sourcen)
# #
# set -Eeuo pipefail

# # ----------------------------- CLI & Globals ----------------------------------
# INSTALL=0
# NO_COLOR=0
# for arg in "$@"; do
#   case "$arg" in
#     --install) INSTALL=1 ;;
#     --no-color) NO_COLOR=1 ;;
#     -h|--help)
#       cat <<'USAGE'
# Verwendung: ros2-env-setup.sh [--install] [--no-color]
#   --install   Kopiert das Skript nach ~/.local/bin/setup-ros2-env und sorgt dafür,
#               dass ~/.local/bin im PATH ist. Danach kann es aus jedem Ordner per
#               'setup-ros2-env' gestartet werden.
#   --no-color  Deaktiviert farbige Ausgabe.
# USAGE
#       exit 0
#       ;;
#     *) echo "Unbekannte Option: $arg"; exit 1 ;;
#   esac
# done

# # ----------------------------- Farben & Output --------------------------------
# USE_COLOR=1
# if [[ "$NO_COLOR" -eq 1 || ! -t 1 ]]; then USE_COLOR=0; fi
# color() { # color '<code>' <text>
#   local c="$1"; shift
#   if [[ $USE_COLOR -eq 1 ]]; then printf "[%sm%s[0m" "$c" "$*"; else printf "%s" "$*"; fi
# }
# info()  { echo -e "$(color '1;32m')✔$(color '0m') $*"; }
# warn()  { echo -e "$(color '1;33m')!$(color '0m') $*"; }
# err()   { echo -e "$(color '1;31m')✖ $*$(color '0m')" >&2; }

# # ----------------------------- Pfad-Utilities ---------------------------------
# get_abs_path() { # robust, ohne externe Abhängigkeiten
#   local target="$1"
#   if command -v readlink >/dev/null 2>&1; then
#     local rp
#     rp="$(readlink -f "$target" 2>/dev/null || true)"
#     [[ -n "$rp" ]] && { echo "$rp"; return; }
#   fi
#   if command -v realpath >/dev/null 2>&1; then
#     realpath "$target" && return
#   fi
#   ( cd "$(dirname -- "$target")" 2>/dev/null && printf '%s/%s' "$(pwd -P)" "$(basename -- "$target")" )
# }
# SCRIPT_PATH="$(get_abs_path "$0")"
# SCRIPT_DIR="$(dirname -- "$SCRIPT_PATH")"
# SCRIPT_NAME="$(basename -- "$SCRIPT_PATH")"

# expand_path() {
#   local p="$1"
#   case "$p" in
#     "~") printf '%s' "$HOME" ;;
#     "~/"*) printf '%s/%s' "$HOME" "${p#~/}" ;;
#     *) printf '%s' "$p" ;;
#   esac
# }

# backup_file() { # <file>
#   local f; f="$(expand_path "$1")"
#   [[ -e "$f" ]] || return 0
#   local ts; ts="$(date +%Y%m%d-%H%M%S)"
#   cp -p "$f" "${f}.bak-${ts}"
#   info "Backup erstellt: ${f}.bak-${ts}"
# }

# upsert_block() { # <file> <block>  (ersetzt bestehenden, markierten Block)
#   local file; file="$(expand_path "$1")"
#   local block; block="$2"
#   local start="# >>> ros2-env-setup (managed) >>>"
#   local end="# <<< ros2-env-setup (managed) <<<"
#   mkdir -p "$(dirname "$file")"
#   touch "$file"
#   backup_file "$file"
#   local tmp; tmp="${file}.tmp.$$"
#   awk -v s="$start" -v e="$end" '
#     BEGIN{skip=0}
#     index($0,s){skip=1; next}
#     index($0,e){skip=0; next}
#     !skip{print}
#   ' "$file" > "$tmp"
#   { cat "$tmp"; echo "$block"; } > "$file"
#   rm -f "$tmp"
# }

# ensure_localbin_in_path() { # schreibt bei Bedarf einen PATH-Block in bash/zsh
#   local block_start="# >>> localbin-path (managed) >>>"
#   local block_end="# <<< localbin-path (managed) <<<"
#   local block
#   block=$(cat <<'EOB'
# # >>> localbin-path (managed) >>>
# # Stellt sicher, dass ~/.local/bin im PATH liegt (für benutzerlokale Tools)
# if [ -d "$HOME/.local/bin" ] && [[ ":$PATH:" != *":$HOME/.local/bin:"* ]]; then
#   export PATH="$HOME/.local/bin:$PATH"
# fi
# # <<< localbin-path (managed) <<<
# EOB
# )
#   upsert_block "$HOME/.bashrc" "$block"
#   upsert_block "$HOME/.zshrc"  "$block"
# }

# # ----------------------------- Standardwerte ----------------------------------
# DEFAULT_DISTRO="humble"
# DEFAULT_LOCALHOST_ONLY="0"   # 0 = Netzwerk erlaubt, 1 = nur localhost
# DEFAULT_DOMAIN_ID=""         # leer = nicht setzen

# # aus DISTRO abgeleitete Defaults; werden nach Nutzereingabe aktualisiert
# DEFAULT_BASH_SETUP="/opt/ros/${DEFAULT_DISTRO}/setup.bash"
# DEFAULT_ZSH_SETUP="/opt/ros/${DEFAULT_DISTRO}/setup.zsh"

# # ----------------------------- Installationsmodus ------------------------------
# if [[ "$INSTALL" -eq 1 ]]; then
#   local_target="$HOME/.local/bin/setup-ros2-env"
#   mkdir -p "$HOME/.local/bin"
#   cp "$SCRIPT_PATH" "$local_target"
#   chmod +x "$local_target"
#   info "Installiert nach: $local_target"
#   ensure_localbin_in_path
#   echo
#   echo "$(color '1;34m')Hinweis:$(color '0m') Öffne eine neue Shell oder führe aus: 'source ~/.bashrc'"
#   echo "Danach kannst du das Tool überall starten mit: $(color '1;36m')setup-ros2-env$(color '0m')"
#   exit 0
# fi

# # ----------------------------- Eingabe-Helfer ---------------------------------
# prompt_default() { # prompt_default "Frage" "Voreinstellung" -> setzt REPLY
#   local q="$1"; local def="${2-}"
#   if [[ -n "$def" ]]; then
#     read -r -p "$q [${def}]: " REPLY || true
#     REPLY=${REPLY:-$def}
#   else
#     read -r -p "$q: " REPLY || true
#   fi
# }

# require_numeric_or_empty() { local v="$1"; [[ -z "$v" || "$v" =~ ^[0-9]+$ ]]; }

# file_exists_note() { # <path>
#   local p; p="$(expand_path "$1")"
#   if [[ -f "$p" ]]; then info "Gefunden: $p"; else warn "Datei nicht gefunden: $p"; fi
# }

# # ----------------------------- Interaktiv -------------------------------------

# echo
# echo "$(color '1;36m')ROS 2 Umgebungssetup$(color '0m') — interaktiver Assistent"

# echo
# prompt_default "Welche ROS 2 Distro soll verwendet werden?" "$DEFAULT_DISTRO"
# DISTRO="$REPLY"

# BASH_SETUP_DEFAULT="/opt/ros/${DISTRO}/setup.bash"
# ZSH_SETUP_DEFAULT="/opt/ros/${DISTRO}/setup.zsh"

# prompt_default "Pfad zu setup.bash (bash)" "$BASH_SETUP_DEFAULT"
# BASH_SETUP_PATH="$REPLY"; file_exists_note "$BASH_SETUP_PATH"

# prompt_default "Pfad zu setup.zsh (zsh)" "$ZSH_SETUP_DEFAULT"
# ZSH_SETUP_PATH="$REPLY"; file_exists_note "$ZSH_SETUP_PATH"

# echo
# echo "Welche Shells sollen konfiguriert werden?"
# echo "  [1] nur bash (~/.bashrc)"
# echo "  [2] bash + zsh (~/.bashrc und ~/.zshrc)"
# prompt_default "Auswahl" "1"
# CHOICE_SHELLS="$REPLY"
# [[ "$CHOICE_SHELLS" =~ ^[12]$ ]] || { err "Ungültige Auswahl."; exit 1; }

# prompt_default "ROS_DOMAIN_ID setzen? (leer = nicht setzen)" "$DEFAULT_DOMAIN_ID"
# ROS_DOMAIN_ID="$REPLY"
# if ! require_numeric_or_empty "$ROS_DOMAIN_ID"; then err "Bitte ganze Zahl oder leer."; exit 1; fi

# prompt_default "ROS_LOCALHOST_ONLY setzen? (0 oder 1)" "$DEFAULT_LOCALHOST_ONLY"
# ROS_LOCALHOST_ONLY="$REPLY"
# [[ "$ROS_LOCALHOST_ONLY" =~ ^[01]$ ]] || { err "Bitte 0 oder 1 angeben."; exit 1; }

# START_MARK="# >>> ros2-env-setup (managed) >>>"
# END_MARK="# <<< ros2-env-setup (managed) <<<"

# mk_block() {
#   local sh="$1"; local spath; spath="$(expand_path "$2")"
#   local lines=()
#   lines+=("$START_MARK")
#   lines+=("# Automatisch verwalteter Block — angepasst mit ros2-env-setup.sh")
#   lines+=("# Shell: $sh | Distro: ${DISTRO}")
#   lines+=("if [ -f \"$spath\" ]; then")
#   lines+=("  source \"$spath\"")
#   lines+=("fi")
#   if [[ -n "$ROS_DOMAIN_ID" ]]; then
#     lines+=("export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}")
#   fi
#   if [[ "$ROS_LOCALHOST_ONLY" == "0" ]]; then
#     lines+=("export ROS_LOCALHOST_ONLY=0")
#   else
#     lines+=("export ROS_LOCALHOST_ONLY=1")
#   fi
#   lines+=("# Prüfen: printenv | grep -i ROS")
#   lines+=("$END_MARK")
#   printf '%s
# ' "${lines[@]}"
# }

# APPLIED_TO=()
# if [[ "$CHOICE_SHELLS" == "1" || "$CHOICE_SHELLS" == "2" ]]; then
#   BRC="$HOME/.bashrc"
#   BLOCK_BASH="$(mk_block bash "$BASH_SETUP_PATH")"
#   upsert_block "$BRC" "$BLOCK_BASH"; APPLIED_TO+=("$BRC")
# fi
# if [[ "$CHOICE_SHELLS" == "2" ]]; then
#   ZRC="$HOME/.zshrc"
#   BLOCK_ZSH="$(mk_block zsh "$ZSH_SETUP_PATH")"
#   upsert_block "$ZRC" "$BLOCK_ZSH"; APPLIED_TO+=("$ZRC")
# fi

# echo
# info "Konfiguration abgeschlossen."
# echo "Aktualisiert wurden: ${APPLIED_TO[*]}"

# echo
# cat <<EOF
# $(color '1;34m')Nächste Schritte:$(color '0m')
#   • Öffne eine neue Shell-Session, ODER führe eines der folgenden Kommandos aus:
#       source ~/.bashrc     # für bash
#       source ~/.zshrc      # für zsh
#   • Prüfe die Umgebung mit:
#       printenv | grep -i ROS

# $(color '1;34m')Aus jedem Ordner starten:$(color '0m')
#   • Optional installieren:
#       ./${SCRIPT_NAME} --install
#     Danach: neue Shell öffnen oder 'source ~/.bashrc' und einfach
#       setup-ros2-env
#     eingeben – unabhängig vom aktuellen Verzeichnis.

# $(color '1;34m')Hinweise:$(color '0m')
#   • Der eingefügte Block ist zwischen Markern:
#         $START_MARK
#         $END_MARK
#     Du kannst ihn später leicht wieder entfernen oder mit diesem Skript überschreiben.
#   • Wenn sich deine ROS-Installation an einem anderen Ort befindet,
#     starte das Skript erneut und gib die korrekten setup-Pfade an.
# EOF
