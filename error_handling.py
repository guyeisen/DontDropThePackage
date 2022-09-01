class PathNotFoundException(Exception):
    """Raised when path could not be made from current sample."""
    pass

class GiveUpException(Exception):
    """Path creation failed for too many times"""
    pass