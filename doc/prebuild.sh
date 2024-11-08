#/bin/sh

mkdir -p $(dirname $0)/bob

curl https://raw.githubusercontent.com/bob-ros2/bob_whisper_cpp/refs/heads/main/README.md > doc/bob/bob-whisper-cpp.md
curl https://raw.githubusercontent.com/bob-ros2/rosgpt4all/refs/heads/main/README.md > doc/bob/rosgpt4all.md
curl https://raw.githubusercontent.com/bob-ros2/bob_topic_tools/refs/heads/main/README.md > doc/bob/bob-topic-tools.md
curl https://raw.githubusercontent.com/bob-ros2/voskros/refs/heads/main/README.md > doc/bob/voskros.md


cat <<EOF > $(dirname $0)/index.rst
Welcome to Bobs's Handbuch!
===================================

.. toctree::
   :maxdepth: 2
   :caption: Packages:

   bob/bob-topic-tools.md
   bob/bob-whisper-cpp.md
   bob/rosgpt4all.md
   bob/voskros.md

EOF
