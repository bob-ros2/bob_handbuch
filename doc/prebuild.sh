#/bin/sh

REPOS="bob_topic_tools bob_launch bob_msgs voskros bob_moondream bob_moondream_msgs bob_coquitts vox dindbox bob_llm bob_vector_db bob_sd35 bob_flux2"
APIS="bob_topic_tools bob_moondream bob_coquitts vox bob_llm bob_vector_db bob_sd35 bob_flux2"

retrieve_readme() {
  curl https://raw.githubusercontent.com/bob-ros2/$1/refs/heads/main/README.md > bob/$(echo $1|sed 's/_/-/g').md
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

# Retrieve all READMEs first
for REPO in $REPOS; do
  retrieve_readme $REPO
done

# Generate sorted package lists
BOB_PACKAGES=$(echo "$REPOS" | tr ' ' '\n' | grep "^bob_" | sed 's/_/-/g' | sort | sed 's/^/   bob\//; s/$/.md/')
VOSK_PACKAGE=$(echo "$REPOS" | tr ' ' '\n' | grep "voskros" | sed 's/_/-/g' | sed 's/^/   bob\//; s/$/.md/')
OTHER_PACKAGES=$(echo "$REPOS" | tr ' ' '\n' | grep -v "^bob_" | grep -v "voskros" | sed 's/_/-/g' | sort | sed 's/^/   bob\//; s/$/.md/')

cat <<EOF > $(dirname $0)/index.rst
Welcome to Bob's Handbuch
=========================
This is a collection of various ROS 2 packages and nodes for natural language processing, LLM integration, and system control. They leverage ROS topics to connect NLP and LLM components seamlessly.

Key Features:

* **LLM Integration**: Compatible Chat Completion API client with function calling capabilities.
* **Speech Processing**: Real-time Speech-to-Text and Text-to-Speech.
* **Vision**: Image-to-Text description.
* **Vector Database**: Text and Image embedding and querying.
* **Utilities**: Topic filtering and routing, simplified launch configuration.
* **Infrastructure**: Docker-in-Docker sandbox environment.

.. raw:: html

   <!-- Add a placeholder for the Twitch embed -->
   <div id="twitch-embed"></div>
   <!-- Load the Twitch embed JavaScript file -->
   <script src="https://embed.twitch.tv/embed/v1.js"></script>
   <!-- Create a Twitch.Embed object that will render within the "twitch-embed" element -->
   <script type="text/javascript">
      new Twitch.Embed("twitch-embed", {
        width: 427,
        height: 440,
        channel: "superbob_6110",
        // Only needed if this page is going to be embedded on other websites
        parent: ["bob-ros2.github.io"]
      });
   </script>
   </br>

.. toctree::
   :maxdepth: 2 
   :caption: Packages

$BOB_PACKAGES
$VOSK_PACKAGE

.. toctree::
   :maxdepth: 2 
   :caption: OTHER

$OTHER_PACKAGES

.. toctree::
   :maxdepth: 2 
   :caption: Miscellaneous

   bob/bob-docker-network.md
   bob/bob-portainer.md

EOF
