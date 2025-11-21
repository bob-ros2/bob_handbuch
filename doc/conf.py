# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Bob\'s Handbuch'
copyright = 'BobRos'
author = 'BobRos'
release = '0.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ['myst_parser', 'autoapi.extension']

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

# Document Python Code
autoapi_type = 'python'
autoapi_dirs = [
    'bob/src/bob_topic_tools/bob_topic_tools',
    'bob/src/bob_moondream/bob_moondream',
    'bob/src/bob_coquitts/bob_coquitts',
    'bob/src/vox/vox',
    'bob/src/bob_llm/bob_llm',
    'bob/src/bob_vector_db',
    ]

html_show_sphinx = False
html_favicon = '_static/profile_image-300x300.png'
html_logo = '_static/bob_vv.gif'
html_css_files = [
    "custom.css"
]
