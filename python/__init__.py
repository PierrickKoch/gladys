try: # relative import
    from .libgladys_python import *
except ValueError:
    from gladys.libgladys_python import *
