#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# Reset "student" user (Ubuntu-only)
# This script deletes user "student" and its home, then recreates it cleanly.
# Run from terminal:  sudo ~/Desktop/reset_student.sh
# Or via .desktop launcher (uses pkexec to ask for admin password).
#
# SECURITY NOTE:
# - By default PASSWORD="" means you'll be prompted interactively.
# - If you prefer a fixed password, set PASSWORD="YourPassword" below.
#   Avoid committing this file with a plaintext password.
# -----------------------------------------------------------------------------

set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "Please run as root. Example: sudo $0"
  exit 1
fi

USER_NAME="student"
PASSWORD=""   # leave empty to be prompted; or set e.g. PASSWORD="Robotics_DEEB"

# --- ask for password if not provided ---
if [[ -z "${PASSWORD}" ]]; then
  read -s -p "New password for '${USER_NAME}': " PW; echo
  read -s -p "Confirm password: " PW2; echo
  [[ "$PW" == "$PW2" ]] || { echo "Passwords do not match"; exit 1; }
  PASSWORD="$PW"
fi

echo "[i] Stopping processes for user '${USER_NAME}' (if any)..."
pkill -u "${USER_NAME}" 2>/dev/null || true

echo "[i] Deleting user '${USER_NAME}' and home (if present)..."
deluser --remove-home "${USER_NAME}" 2>/dev/null || true
deluser --remove-all-files "${USER_NAME}" 2>/dev/null || true
groupdel "${USER_NAME}" 2>/dev/null || true

echo "[i] Creating fresh user '${USER_NAME}'..."
adduser --disabled-password --gecos "" "${USER_NAME}"

echo "[i] Setting password..."
echo "${USER_NAME}:${PASSWORD}" | chpasswd

echo "[i] Ensuring no sudo privileges..."
deluser "${USER_NAME}" sudo 2>/dev/null || true

# Optional: reset a minimal skeleton (already applied by adduser)
# You could copy templates here if you want a custom default profile.

echo "[✓] User '${USER_NAME}' reset completed."
echo "[i] You can now log in as '${USER_NAME}'."
