#!/usr/bin/env python3
import os
import mistune

# def make_markedown(directory, filename):
#     readme = os.path.join(directory, filename)
#     if not os.path.isfile(readme):
#         return

#     # Open an output file
#     f = open(os.path.join(directory, "html.txt"), 'w+')

#     # Add the text from the README
#     readme = os.path.join(directory, filename)
#     with open(readme, 'r') as readme_f:
#         renderer = mistune.Renderer(hard_wrap=False,escape=False)
#         markdown = mistune.Markdown(renderer=renderer)
#         data = readme_f.read()
#         f.write(markdown(data))

def make_markdown(directory, filename):
    readme_path = os.path.join(directory, filename)
    if not os.path.isfile(readme_path):
        return

    # Open an output file
    with open(os.path.join(directory, "html.txt"), 'w+') as f:
        # Add the text from the README
        with open(readme_path, 'r') as readme_file:
            markdown_text = readme_file.read()
            html_output = mistune.create_markdown(renderer=mistune.HTMLRenderer())(markdown_text)
            f.write(html_output)
        
if __name__ == "__main__":
    # make_markedown(os.getcwd(), "README.md")
    make_markdown(os.getcwd(), "README.md")



