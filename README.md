# photobot

## Installation

```bash
git submodule update --init --recursive shotcut-mcp   # fetch the shotcut-mcp submodule source
python3 shotcut-mcp/scripts/shotcut_mcp_server.py --check   # verify the server starts
claude mcp add --transport stdio shotcut -- python3 shotcut-mcp/scripts/shotcut_mcp_server.py   # register the server for this project only
claude mcp list   # confirm the shotcut server is registered
```
