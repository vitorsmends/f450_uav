#!/bin/bash

set -e

echo "========================================="
echo " Creating Python virtual environment"
echo "========================================="

python3 -m venv .venv

echo "========================================="
echo " Activating virtual environment"
echo "========================================="

source .venv/bin/activate

echo "========================================="
echo " Upgrading pip"
echo "========================================="

pip install --upgrade pip

echo "========================================="
echo " Installing requirements"
echo "========================================="

pip install -r requirements.txt

echo "========================================="
echo " Installation finished"
echo "========================================="

echo ""
echo "To activate the environment:"
echo "source .venv/bin/activate"
echo ""

echo "Testing installation..."
onshape-to-robot --help
