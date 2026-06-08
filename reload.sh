git pull
docker build -t dnonakolesax/cccad-3dk:0.0.1 .
cd /root/cad-docker/cad-sketcher && docker-compose down
cd /root/cad-docker/cad-kernel && docker-compose down && docker-compose up -d
cd /root/cad-docker/cad-sketcher && docker-compose up -d