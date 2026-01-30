# Setup Portainer with Multiple Docker

A `Portainer` setup quick guide.
* [https://www.portainer.io](https://www.portainer.io)

## Setup Portainer on a Docker host
This is the setup for a Synology NAS. It will also act as a web proxy for the `Portainer` Web-GUI.
It should work in a similar way also on any other Docker host.

```bash
# Docker must already be setup on Synology NAS
# Create the folder /volume1/docker/portainer-ce
# ssh login as admin and run the following container
sudo docker run -p 8008:8000 -p 9000:9009 -d \
    --name=portainer-ce --restart=always \
    -v /var/run/docker.sock:/var/run/docker.sock \
    -v /volume1/docker/portainer-ce:/data \
    portainer/portainer-ce:latest
```
Alternatively a docker compose.yaml could look like this:

```yaml
services:
  portainer-ce: 
    image: portainer/portainer-ce:latest
    ports:
      - 8008:8000
      - 9009:9000 
    volumes:
      - /var/run/docker.sock:/var/run/docker.sock
      - /volume1/docker/portainer-ce:/data
    restart: always
```
```bash
# When using ssh, in the directory where the compose.yaml is located
sudo docker compose up -d
```
* Afterwards access the GUI to create an inital admin user: `http://synology-ip:9009`
* When it's done you should see a page where you can select local environment or add another environment
* Choose local environment and now you can manage your Docker host

## Setup and attach a Portainer Agent running on another Docker host

```bash
# Run Portainer Agent Container
sudodocker run -d -p 9001:9001 \
    --name portainer-agent --restart=always \
    -v /var/run/docker.sock:/var/run/docker.sock \
    -v /var/lib/docker/volumes:/var/lib/docker/volumes \
    portainer/agent:latest
```

* If the container could be started choose in the Portainer GUI: `Enviroment-related -> Environments`
* Click `+ Add Environment`
* In the upcoming wizard choose `Docker Standalone` 
* Click `Start Wizard`

* Give it a name and an environment address in the format e.g.:
`192.168.1.123:9001`
* Afterwards click `Connect` and if it works the new environment will be added. 
* Finally click `Close` and go to the Start Page

Et voilà, you can see now two Docker hosts.