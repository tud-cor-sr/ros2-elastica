__all__ = ["MuscleTorquesWithVaryingBetaSplines"]

from elastica import IMPORT_NUMBA

if IMPORT_NUMBA:
    from examples.ContinuumSnakeCase.muscle_torques_with_bspline_numba import (
        MuscleTorquesWithVaryingBetaSplines,
    )

else:
    from examples.ContinuumSnakeCase.muscle_torques_with_bspline_numpy import (
        MuscleTorquesWithVaryingBetaSplines,
    )
