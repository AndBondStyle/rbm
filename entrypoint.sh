#!/bin/bash
if [ -f /src/.autostart ]; then tmuxp load -d /src; fi
trap : TERM INT; sleep infinity & wait
