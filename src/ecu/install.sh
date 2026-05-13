#!/bin/bash 
set -e

log() {
  echo "[install] $*"
}

if [ "${EUID:-$(id -u)}" -ne 0 ]; then
  echo "This script must be run as root." >&2
  exit 1
fi

log "Starting turtlebot ECU install"

ecu_user="$(awk -F= '/^User=/{print $2; exit}' ./services/tb_ecu_web.service | tr -d '\r[:space:]')"
if [ -z "$ecu_user" ]; then
  ecu_user="$(printf '%s' "${SUDO_USER:-${USER:-}}" | tr -d '\r[:space:]')"
fi

if [ -n "$ecu_user" ] && [ "$ecu_user" != "root" ] && command -v sudo >/dev/null 2>&1; then
  old_sudoers_file="/etc/sudoers.d/tb_ecu_web_poweroff"
  sudoers_file="/etc/sudoers.d/99-tb_ecu_web_poweroff"
  helper_path="/usr/local/sbin/tb_ecu_poweroff"
  systemctl_path="$(command -v systemctl)"
  log "Configuring poweroff permission for $ecu_user"

  {
    echo "#!/bin/sh"
    echo "exec $systemctl_path --no-block poweroff"
  } > "$helper_path"
  chown root:root "$helper_path"
  chmod 0755 "$helper_path"
  rm -f "$old_sudoers_file"

  {
    echo "# Allow ECU web UI to power off turtlebot without a password."
    echo "$ecu_user ALL=(root) NOPASSWD: $helper_path"
    echo "$ecu_user ALL=(ALL) NOPASSWD: $helper_path"
  } > "$sudoers_file"
  chmod 0440 "$sudoers_file"
  if ! visudo -q -cf "$sudoers_file"; then
    rm -f "$sudoers_file"
    echo "Invalid sudoers file for $ecu_user; removed $sudoers_file." >&2
    exit 1
  fi
  if ! sudo -u "$ecu_user" sudo -n -l "$helper_path" >/dev/null 2>&1; then
    log "Warning: poweroff permission validation failed for $ecu_user"
    log "Check manually: sudo -u $ecu_user sudo -n -l $helper_path"
  else
    log "Poweroff permission OK"
  fi
else
  log "Skipping poweroff sudoers setup"
fi

services=()
while IFS= read -r service; do
  [ -n "$service" ] && services+=("$service")
done < <(systemctl list-unit-files --type=service --no-legend --no-pager | awk '{print $1}' | grep '^tb_.*\.service$' || true)

if [ "${#services[@]}" -eq 0 ]; then
  log "No installed tb_ services found; using local service files"
  while IFS= read -r service_file; do
    services+=("$(basename "$service_file")")
  done < <(find ./services -maxdepth 1 -type f -name 'tb_*.service' | sort)
fi

if [ "${#services[@]}" -gt 0 ]; then
  log "Stopping services: ${services[*]}"
  systemctl stop "${services[@]}"
fi

log "Installing service files"
cp -f ./services/* /etc/systemd/system/
log "Reloading systemd"
systemctl daemon-reload

if [ "${#services[@]}" -gt 0 ]; then
  log "Starting services: ${services[*]}"
  systemctl start "${services[@]}"
fi

log "Done"
