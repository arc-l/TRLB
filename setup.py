from pip._internal.req import parse_requirements
from setuptools import setup, find_packages

# Get required packages
install_reqs = parse_requirements("requirements.txt", session=False)
reqs = [str(ir.requirement) for ir in install_reqs]

# Setup
setup(name="TRLB", version="1.0", packages=find_packages(), install_requires=reqs)