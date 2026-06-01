#!/bin/bash
# NOTE: el_a3_web_ui was removed from this repo; this script no longer starts a server.
# EL-A3 Web UI Startup Script (historical; see message below)

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}"
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║         EL-A3 Web Control Interface                       ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Default parameters
HOST="0.0.0.0"
PORT="5000"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --host)
            HOST="$2"
            shift 2
            ;;
        --port)
            PORT="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [--host HOST] [--port PORT]"
            echo "  --host HOST  Bind address (default: 0.0.0.0)"
            echo "  --port PORT  Port number (default: 5000)"
            exit 0
            ;;
        *)
            shift
            ;;
    esac
done

echo -e "${YELLOW}el_a3_web_ui is no longer in this workspace.${NC}"
echo "If you need a web UI, add a replacement package or run it from a separate project."
echo "Requested host=$HOST port=$PORT (not used)."
exit 1
