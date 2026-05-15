#!/bin/bash

set -e

echo "Creating Python virtual environment..."

python3 -m venv .venv

echo "Activating virtual environment..."

source .venv/bin/activate

echo "Upgrading pip..."

pip install --upgrade pip

echo "Installing dependencies..."

pip install -r requirements.txt

echo ""
echo "Setup completed successfully."
echo ""
echo "To activate the environment later, run:"
echo "source .venv/bin/activate"