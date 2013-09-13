try: # relative import
    from .libgladys_python import *
except ValueError:
    from libgladys_python import *
