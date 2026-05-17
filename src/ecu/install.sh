#!/bin/bash 
set -e

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"

log() {
  echo "[install] $*"
}

apt_install() {
  DEBIAN_FRONTEND=noninteractive apt-get install -y "$@"
}

apt_package_available() {
  apt-cache show "$1" >/dev/null 2>&1
}

install_ldlidar_udev_rules() {
  local rules_src="./ros2_ws/src/ldrobot-lidar-ros2/rules/ldlidar.rules"
  local rules_dst="/etc/udev/rules.d/ldlidar.rules"

  if [ ! -f "$rules_src" ]; then
    log "Skipping LDRobot udev rules; $rules_src was not found"
    return
  fi

  log "Installing LDRobot udev rules"
  cp -f "$rules_src" "$rules_dst"
  chmod 0644 "$rules_dst"

  if command -v udevadm >/dev/null 2>&1; then
    udevadm control --reload-rules || true
    udevadm trigger || true
  fi

  if systemctl list-unit-files udev.service >/dev/null 2>&1; then
    systemctl restart udev || true
  elif command -v service >/dev/null 2>&1; then
    service udev restart || true
  fi
}

install_docker_apt_repo() {
  . /etc/os-release

  docker_os_id="${ID:-}"
  docker_codename="${VERSION_CODENAME:-${UBUNTU_CODENAME:-}}"

  if [ "$docker_os_id" != "ubuntu" ] && [ "$docker_os_id" != "debian" ]; then
    echo "Unsupported OS for automatic Docker repo setup: ${PRETTY_NAME:-unknown}." >&2
    exit 1
  fi

  if [ -z "$docker_codename" ]; then
    echo "Could not detect OS codename for Docker repo setup." >&2
    exit 1
  fi

  log "Configuring Docker apt repository"
  apt-get update
  apt_install ca-certificates curl gnupg
  install -m 0755 -d /etc/apt/keyrings
  curl -fsSL "https://download.docker.com/linux/$docker_os_id/gpg" -o /etc/apt/keyrings/docker.asc
  chmod a+r /etc/apt/keyrings/docker.asc

  {
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/$docker_os_id $docker_codename stable"
  } > /etc/apt/sources.list.d/docker.list
  apt-get update
}

install_docker_compose_binary() {
  case "$(uname -m)" in
    x86_64)
      compose_arch="x86_64"
      ;;
    aarch64|arm64)
      compose_arch="aarch64"
      ;;
    armv7l)
      compose_arch="armv7"
      ;;
    *)
      echo "Unsupported architecture for automatic Docker Compose install: $(uname -m)." >&2
      exit 1
      ;;
  esac

  log "Installing Docker Compose plugin from official release binary"
  apt-get update
  apt_install ca-certificates curl
  install -m 0755 -d /usr/local/lib/docker/cli-plugins
  curl -fL "https://github.com/docker/compose/releases/latest/download/docker-compose-linux-$compose_arch" \
    -o /usr/local/lib/docker/cli-plugins/docker-compose
  chmod 0755 /usr/local/lib/docker/cli-plugins/docker-compose
}

ensure_docker() {
  if ! command -v docker >/dev/null 2>&1; then
    log "Installing Docker"
    apt-get update
    if apt_package_available docker.io; then
      apt_install docker.io
    else
      install_docker_apt_repo
      apt_install docker-ce docker-ce-cli containerd.io
    fi
  fi

  if ! docker compose version >/dev/null 2>&1; then
    log "Installing Docker Compose plugin"
    apt-get update
    if apt_package_available docker-compose-plugin; then
      apt_install docker-compose-plugin
    elif apt_package_available docker-compose-v2; then
      apt_install docker-compose-v2
    else
      install_docker_compose_binary
    fi
  fi

  systemctl enable --now docker

  if ! docker compose version >/dev/null 2>&1; then
    echo "docker compose is required to start the ROS2 container." >&2
    exit 1
  fi
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

nginx_config_src="./config/turtlebot_nginx.conf"

if ! command -v nginx >/dev/null 2>&1; then
  log "Installing nginx"
  apt-get update
  DEBIAN_FRONTEND=noninteractive apt-get install -y nginx
fi

log "Configuring nginx reverse proxy"
cp -f "$nginx_config_src" /etc/nginx/sites-available/turtlebot
ln -sf /etc/nginx/sites-available/turtlebot /etc/nginx/sites-enabled/turtlebot
rm -f /etc/nginx/sites-enabled/default
nginx -t
systemctl enable nginx
systemctl restart nginx
log "nginx is serving turtlebot on port 80"

ensure_docker

install_ldlidar_udev_rules

log "Recreating ROS2 docker container"
(
  cd ./docker
  docker compose down --remove-orphans
  docker rm -f ros2_humble_tx2 >/dev/null 2>&1 || true
  docker compose build
  docker compose up -d
)
log "ROS2 docker container is running"

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

ln -snf /home/"$ecu_user"/releases/active_release/ /home/"$ecu_user"/active_release

log "Done"
