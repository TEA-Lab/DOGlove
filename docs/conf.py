# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'DOGlove'
copyright = '2025, DOGlove'
author = 'Han Zhang, Songbo Hu, Zhecheng Yuan, Huazhe Xu'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    # Sphinx's own extensions
    "sphinx.ext.autodoc",
    "sphinx.ext.extlinks",
    "sphinx.ext.intersphinx",
    "sphinx.ext.mathjax",
    "sphinx.ext.todo",
    "sphinx.ext.viewcode",
    'myst_parser',
]

exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "furo"
html_title = "DOGlove"
html_logo = "_static/logo.png"
language = "en"

html_static_path = ['_static']
html_css_files = ["pied-piper-admonition.css"]

# -- Options for Autodoc --------------------------------------------------------------

autodoc_member_order = "bysource"
autodoc_preserve_defaults = True

# Keep the type hints outside the function signature, moving them to the
# descriptions of the relevant function/methods.
autodoc_typehints = "description"

# Allow both .rst and .md
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}
