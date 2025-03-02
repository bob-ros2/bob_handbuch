#/bin/sh

REPOS="bob_llama_cpp bob_topic_tools bob_launch bob_whisper_cpp bob_msgs bob_transformers rosgpt4all voskros"
APIS="bob_llama_cpp bob_topic_tools bob_transformers rosgpt4all"

retrieve_readme() {
  curl https://raw.githubusercontent.com/bob-ros2/$1/refs/heads/main/README.md > doc/bob/$(echo $1|sed 's/_/-/g').md
}

retrieve_src() {
  git clone --filter=blob:none https://github.com/bob-ros2/$1.git
  cd $1
  git sparse-checkout init
  git sparse-checkout set $1
  git checkout main
  cd -
}
mkdir -p $(dirname $0)/bob/src
WORKDIR=$(pwd)
cd $(dirname $0)/bob/src
for API in $APIS; do
  retrieve_src $API
done
cd $WORKDIR

cat <<EOF > $(dirname $0)/index.rst
Welcome to Bob's Handbuch
=========================
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
   :caption: Packages

$(
   for REPO in $REPOS; do
      retrieve_readme $REPO
      echo "   bob/$(echo $REPO|sed 's/_/-/g').md"
   done
)

.. toctree::
   :maxdepth: 2 
   :caption: Miscellaneous

   bob/bob-docker-network.md
   bob/bob-portainer.md
   bob/bob-doxygen.md

EOF
