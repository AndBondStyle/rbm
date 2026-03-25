### jupyter

```
pip install jupyterlab

mkdir -p /root/venv/share/jupyter/lab/settings
nano /root/venv/share/jupyter/lab/settings/overrides.json

### PASTE
{
  "@jupyterlab/apputils-extension:themes": {
    "theme": "JupyterLab Dark"
  }
}
###

mkdir -p /src/jupyter
cd /src/jupyter
jupyter lab --allow-root --ip=0.0.0.0 --port=8080 --no-browser --ServerApp.token='' --ServerApp.password=''
```
