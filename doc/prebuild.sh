#/bin/sh

REPOS="bob_llama_cpp bob_topic_tools bob_whisper_cpp bob_msgs rosgpt4all voskros"

retrieve_readme() {
   curl https://raw.githubusercontent.com/bob-ros2/$1/refs/heads/main/README.md > doc/bob/$(echo $1|sed 's/_/-/g').md
}

mkdir -p $(dirname $0)/bob

cat <<EOF > $(dirname $0)/index.rst
Welcome to Bob's Handbuch
===================================
This is a collection of various ROS packages and nodes for natural language processing and system control. They make use of the ROS topics to connect the NLP components.

* Compatible Chat Completion API client 
* API client with function calling cababilities
* Speach to text
* Text to speach
* Text to image
* Image to text
* Topic String message filtering and routing
* Text embedding
* Image embedding
* Querying Vector databases
* Easy ROS launch config in yaml format

.. toctree::
   :maxdepth: 2 
   :caption: Packages:

$(
   for REPO in $REPOS; do
      retrieve_readme $REPO
      echo "   bob/$(echo $REPO|sed 's/_/-/g').md"
   done
)
   bob/bob-docker-network.md
   bob/bob-portainer

EOF
