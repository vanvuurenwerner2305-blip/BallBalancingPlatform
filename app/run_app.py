"""Start the Ball Balancing Platform app:  python app/run_app.py"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from bbp_app.main import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
