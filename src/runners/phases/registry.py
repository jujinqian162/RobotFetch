from __future__ import annotations

from workflow.phase_runner import PhaseRunner
from workflow.types import Phase

from .base_coord_phase import BaseCoordPhaseRunner
from .forward_approach_phase import ForwardApproachPhaseRunner
from .status_align_phase import StatusAlignPhaseRunner


def build_phase_registry() -> dict[Phase, PhaseRunner]:
    return {
        Phase.STATUS_ALIGN: StatusAlignPhaseRunner(),
        Phase.FORWARD_APPROACH: ForwardApproachPhaseRunner(),
        Phase.BASE_COORD: BaseCoordPhaseRunner(),
    }
