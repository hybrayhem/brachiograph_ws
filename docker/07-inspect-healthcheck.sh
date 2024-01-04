#!/bin/bash

# echo "Inspecting health of brachiograph-sim..."
# docker inspect brachiograph-sim --format "{{range .State.Health.Log}}{{.End}} | Exit Code: {{.ExitCode}} | {{.Output}}{{end}}"

echo "Inspecting health of brachiograph-novnc..."
docker inspect brachiograph-novnc --format "{{range .State.Health.Log}}{{.End}} | Exit Code: {{.ExitCode}} | {{.Output}}{{end}}"