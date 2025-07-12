Welcome to Bob's Handbuch
=========================
This is a collection of various ROS packages and nodes for natural language processing and system control. They make use of the ROS topics to connect the NLP and LLM components.

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

   bob/bob-llama-cpp.md
   bob/bob-topic-tools.md
   bob/bob-launch.md
   bob/bob-whisper-cpp.md
   bob/bob-msgs.md
   bob/bob-transformers.md
   bob/bob-vector-db.md
   bob/rosgpt4all.md
   bob/voskros.md

.. toctree::
   :maxdepth: 3 
   :caption: Miscellaneous

   bob/bob-docker-network.md
   bob/bob-portainer.md
   bob/bob-doxygen.md

