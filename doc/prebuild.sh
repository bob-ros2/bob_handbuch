#/bin/sh

REPOS="bob_whisper_cpp rosgpt4all bob_topic_tools voskros"

retrieve_readme() {
   curl https://raw.githubusercontent.com/bob-ros2/$1/refs/heads/main/README.md > doc/bob/$(echo $1|sed 's/_/-/g').md
}

mkdir -p $(dirname $0)/bob

cat <<EOF > $(dirname $0)/index.rst
Welcome to Bobs's Handbuch!
===================================

.. toctree::
   :maxdepth: 2
   :caption: Packages:

$(
   for REPO in $REPOS; do
      retrieve_readme $REPO
      echo "   bob/$(echo $REPO|sed 's/_/-/g').md"
   done
)

EOF
