#!/usr/bin/env bash
set -euo pipefail

# ========= CẤU HÌNH =========
DB_USER="${DB_USER:-chuongnv}"
DB_PASS="${DB_PASS:-3vmlkH!zTju5hh7v}"

NVM_VERSION="${NVM_VERSION:-v0.40.4}"
NODE_VERSION="${NODE_VERSION:-22}"

log() { echo -e "\n\033[1;32m[INFO]\033[0m $*"; }
warn(){ echo -e "\n\033[1;33m[WARN]\033[0m $*"; }

# ========= 1) UPDATE/UPGRADE =========
log "apt update/upgrade/autoremove"
sudo apt update
sudo apt upgrade -y
sudo apt autoremove -y

# ========= 2) APACHE + PHP =========
log "Cài apache2"
sudo apt install -y apache2
log "Enable + start apache2"
sudo systemctl enable --now apache2

log "Cài PHP + module Apache + php-mysql"
sudo apt install -y php libapache2-mod-php php-mysql
log "Restart apache2"
sudo systemctl restart apache2

# ========= 3) MARIADB =========
log "Cài mariadb-server"
sudo apt install -y mariadb-server
log "Enable + start mariadb"
sudo systemctl enable --now mariadb

# Tự động hóa mysql_secure_installation
log "Cài expect để tự động chạy mysql_secure_installation"
sudo apt install -y expect

log "Chạy mysql_secure_installation (tự trả lời theo cấu hình bạn đưa)"
sudo expect <<'EOF'
set timeout -1
spawn sudo mysql_secure_installation

expect {
  -re "Enter current password for root.*:" { send "\r"; exp_continue }
  -re "Switch to unix_socket authentication.*\\(Y/n\\)" { send "y\r"; exp_continue }
  -re "Change the root password\\? \\(Y/n\\)" { send "n\r"; exp_continue }
  -re "Remove anonymous users\\? \\(Y/n\\)" { send "y\r"; exp_continue }
  -re "Disallow root login remotely\\? \\(Y/n\\)" { send "y\r"; exp_continue }
  -re "Remove test database and access to it\\? \\(Y/n\\)" { send "y\r"; exp_continue }
  -re "Reload privilege tables now\\? \\(Y/n\\)" { send "y\r"; exp_continue }
  eof
}
EOF

# ========= 4) TẠO USER SQL =========
log "Tạo user MariaDB: '$DB_USER'@'localhost' và cấp quyền"
sudo mysql -e "CREATE USER IF NOT EXISTS '${DB_USER}'@'localhost' IDENTIFIED BY '${DB_PASS}';"
sudo mysql -e "GRANT ALL PRIVILEGES ON *.* TO '${DB_USER}'@'localhost' WITH GRANT OPTION;"
sudo mysql -e "FLUSH PRIVILEGES;"

# ========= 5) PHPMYADMIN (AUTO) =========
log "Cài phpMyAdmin (dbconfig-common=yes, app-pass=trống, webserver=apache2)"

# Preseed debconf để khỏi hỏi
echo "phpmyadmin phpmyadmin/reconfigure-webserver multiselect apache2" | sudo debconf-set-selections
echo "phpmyadmin phpmyadmin/dbconfig-install boolean true" | sudo debconf-set-selections

# App password: để trống (Enter)
echo "phpmyadmin phpmyadmin/mysql/app-pass password" | sudo debconf-set-selections
echo "phpmyadmin phpmyadmin/app-password-confirm password" | sudo debconf-set-selections

# Admin pass: để trống (root thường unix_socket)
echo "phpmyadmin phpmyadmin/mysql/admin-pass password" | sudo debconf-set-selections

sudo DEBIAN_FRONTEND=noninteractive apt install -y phpmyadmin

log "Restart apache2"
sudo systemctl restart apache2

# ========= 6) NVM + NODE =========
log "Cài NVM ${NVM_VERSION}"
curl -o- "https://raw.githubusercontent.com/nvm-sh/nvm/${NVM_VERSION}/install.sh" | bash

# Load nvm trong shell hiện tại để dùng ngay
export NVM_DIR="${HOME}/.nvm"
# shellcheck disable=SC1091
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh"

log "Cài Node.js ${NODE_VERSION} bằng nvm"
nvm install "${NODE_VERSION}"

# ========= 7) PM2 =========
log "Cài pm2 global"
npm install -g pm2

log "HOÀN TẤT!"
echo "phpMyAdmin: http://<IP>/phpmyadmin"
echo "DB user: ${DB_USER} / pass: ${DB_PASS}"
echo "Node: $(node -v 2>/dev/null || true)"
echo "PM2:  $(pm2 -v 2>/dev/null || true)"
