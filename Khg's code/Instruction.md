# Planning

- Framework:
  - Frontend: Matplotlib, Plotly, PyQt
  - Backend: Numpy, ...
  - Graph: Matplotlib, Plotly, PyQtGraph.OpenGL
  - Data: ruamel.YAML

[Animated 3D Sine Plot](https://gist.github.com/markjay4k/da2f55e28514be7160a7c5fbf95bd243)

[PyQtGraph.OpenGL.GLViewWidget](https://pyqtgraph.readthedocs.io/en/latest/api_reference/3dgraphics/glviewwidget.html)

- Structure:

  - main.py / app.py -> Frontend, Hub -> from engine import inverse / from engine.forward import function
  - position.yaml
  - Folder for processing "engine": inverse.py, forward.py,...
  - test.py

# Writing Code

- Before doing math, write down all the functions you need in the order you need to compute
- Only use class when you need more instances of the same object

# How to use git

.gitignore -> file that determines which file is not uploaded to your repository when using **git push**
How to create gitignore
Press Ctrl + Shift + P -> to open preferences
Select Add gitignore
Choose Python language, it will automatically remove

git tutorial
