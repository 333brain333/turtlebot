#!/bin/bash 
if [ "${EUID:-$(id -u)}" -ne 0 ]; then
  echo "This script must be run as root." >&2
  exit 1
fi

services=()
while IFS= read -r service; do
  [ -n "$service" ] && services+=("$service")
done < <(systemctl list-unit-files --type=service --no-legend --no-pager | awk '{print $1}' | grep '^tb_.*\.service$' || true)

if [ "${#services[@]}" -gt 0 ]; then
  systemctl stop "${services[@]}"
fi

cp -f ./services/* /etc/systemd/system/
systemctl daemon-reload

if [ "${#services[@]}" -gt 0 ]; then
  systemctl start "${services[@]}"
fi
