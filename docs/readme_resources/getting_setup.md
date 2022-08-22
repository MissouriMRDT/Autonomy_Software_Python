# Getting Set Up
[Back to README.md](../../README.md)

1. Clone the repo in an appropriate place. Standard practice is to clone it inside a directory such as RoverSoftware.

```
git clone --recurse-submodules https://github.com/MissouriMRDT/Autonomy_Software.git
```

2. Install Python 3.8 or higher, and then install pipenv using pip

```
pip install pipenv
```

3. Configure your python virtual environment (using pipenv) to install the required dev packages

```
pipenv install -d
```

4. Now that you have configured the virtual environment, if you want to run python code for the autonomy repo make sure
   to always do one of the following:

```
pipenv run <file you want to run>
```

or the preferred option which will spawn a shell subprocess with which you can start running commands within the
environment:

```
pipenv shell
```

5. You should now be set up (bar any issues), so go ahead and run our unit tests or linter using the following commands:

```
pytest --cov # unit tests and coverage

flake8 # linter used for code quality
```

These packages will be run by default for any push to dev, release/testing, or release/competition and any pull request
into dev, release/testing, or release/competition to ensure the code quality matches our standards.

6. To run the autonomy main code:

```
python run.py --file <file to run (you can ignore this if you want to run main autonomy)> --vision <change this to WEBCAM or NONE if you are running this on personal computer>
```

Any of the ZED (our chosen Stereo Cam) specific code will require
the [ZED SDK](https://www.stereolabs.com/developers/release/), though you won't be able to run most code without an
NVIDIA GPU. So use the --vision parameter if you don't have a camera or are using a webcam.

7. If you want to try running autonomy code in a simulator, check
   out [this repo](https://github.com/MissouriMRDT/Autonomy_Simulator) and associated documentation.