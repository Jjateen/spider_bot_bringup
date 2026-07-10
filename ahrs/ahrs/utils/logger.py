import logging
import sys


def setup_logger(name: str, level: int = logging.INFO) -> logging.Logger:
    logger = logging.getLogger(name)
    logger.setLevel(level)

    if logger.handlers:
        return logger

    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(level)

    formatter = logging.Formatter(
        "[%(levelname)s] %(name)s: %(message)s"
    )
    handler.setFormatter(formatter)
    logger.addHandler(handler)

    return logger
