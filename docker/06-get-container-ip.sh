docker inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}, {{end}}' brachiograph-sim