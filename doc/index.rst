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

   bob/bob-coquitts.md
   bob/bob-launch.md
   bob/bob-llm.md
   bob/bob-moondream.md
   bob/bob-moondream-msgs.md
   bob/bob-msgs.md
   bob/bob-topic-tools.md
   bob/bob-vector-db.md
   bob/voskros.md

.. toctree::
   :maxdepth: 2 
   :caption: OTHER

   bob/dindbox.md
   bob/vox.md

.. toctree::
   :maxdepth: 2 
   :caption: Miscellaneous

   bob/bob-docker-network.md
   bob/bob-portainer.md

