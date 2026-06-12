"""
Parse one RTLS host-link line into a typed packet.

Wire format (see firmware HostLink.h / Tag.ino):

  RTLS frames (localization sweeps):
    RTLS,v1,<t_ms>,<tag_id>,<n>,<id1>,<d1_mm>,<q1>,...,<idN>,<dN_mm>,<qN>
         [,IMU,<qw>,<qx>,<qy>,<qz>,<ax>,<ay>,<az>]

  Survey frames (anchor self-survey):
    SURVEY_BEGIN,v1,<n_pairs>
    SURVEY,v1,<src_id>,<dst_id>,<avg_dist_mm>,<ok_samples>
    SURVEY_DONE,v1
"""

from dataclasses import dataclass, field
from typing import List, Optional
import math


# ── IMU / Sweep data-classes ──────────────────────────────────────────────────

@dataclass
class ImuData:
    quat: List[float]   # [w, x, y, z]
    acc:  List[float]   # [ax, ay, az]


@dataclass
class SweepPacket:
    valid: bool = False
    t_ms:  int = 0
    tag_id: int = 0
    ids:   List[int]   = field(default_factory=list)
    dist:  List[float] = field(default_factory=list)
    q:     List[float] = field(default_factory=list)
    imu:   Optional[ImuData] = None


# ── Survey data-classes ───────────────────────────────────────────────────────

@dataclass
class SurveyPair:
    """One pairwise measurement from a SURVEY,v1,… line."""
    src:    int
    dst:    int
    dist_m: float
    ok:     int


@dataclass
class SurveyPacket:
    """
    Represents a single parsed survey line.

    kind: 'begin' | 'pair' | 'done' | None  (None = not a survey line)
    """
    kind:     Optional[str] = None   # 'begin', 'pair', 'done'
    n_pairs:  int = 0                # set on 'begin'
    pair:     Optional[SurveyPair] = None   # set on 'pair'


# ── Parser ────────────────────────────────────────────────────────────────────

class FrameParser:
    """
    Static parser for all host-link frame types emitted by the tag firmware.

    Usage::

        pkt = FrameParser.parse(line)          # SweepPacket — RTLS data
        spkt = FrameParser.parse_survey(line)  # SurveyPacket — survey data
    """

    @staticmethod
    def parse(line: str) -> SweepPacket:
        """Parse an RTLS,v1,… sweep line. Returns SweepPacket(valid=False) on error."""
        pkt = SweepPacket()
        if not line:
            return pkt
        tok = line.strip().split(',')
        if len(tok) < 5 or tok[0] != 'RTLS':
            return pkt

        try:
            pkt.t_ms   = int(tok[2])
            pkt.tag_id = int(tok[3])
            n          = int(tok[4])
        except (ValueError, IndexError):
            return pkt
        if n < 0:
            return pkt

        need = 5 + 3 * n
        if len(tok) < need:
            return pkt

        ids, dist, q = [], [], []
        k = 5
        for _ in range(n):
            try:
                ids.append(int(tok[k]))
                dist.append(float(tok[k + 1]) / 1000.0)   # mm → m
                q.append(float(tok[k + 2]))
            except (ValueError, IndexError):
                return pkt
            k += 3

        pkt.ids  = ids
        pkt.dist = dist
        pkt.q    = q

        # Optional IMU tail
        if len(tok) >= k + 1 and tok[k] == 'IMU' and len(tok) >= k + 8:
            try:
                vals = [float(tok[k + 1 + i]) for i in range(7)]
                pkt.imu = ImuData(quat=vals[:4], acc=vals[4:])
            except ValueError:
                pass

        # Validate: no NaN anywhere
        if any(math.isnan(v) for v in dist + q):
            return pkt
        pkt.valid = True
        return pkt

    @staticmethod
    def parse_survey(line: str) -> SurveyPacket:
        """
        Parse a SURVEY_BEGIN / SURVEY / SURVEY_DONE line.
        Returns SurveyPacket(kind=None) if the line is not a survey frame.
        """
        spkt = SurveyPacket()
        if not line:
            return spkt
        s = line.strip()

        if s.startswith('SURVEY_BEGIN,v1,'):
            tok = s.split(',')
            try:
                spkt.kind    = 'begin'
                spkt.n_pairs = int(tok[2])
            except (IndexError, ValueError):
                pass
            return spkt

        if s.startswith('SURVEY_DONE'):
            spkt.kind = 'done'
            return spkt

        if s.startswith('SURVEY,v1,'):
            tok = s.split(',')
            if len(tok) >= 6:
                try:
                    src    = int(tok[2])
                    dst    = int(tok[3])
                    dist_m = float(tok[4]) / 1000.0
                    ok     = int(tok[5])
                    if ok > 0 and dist_m > 0:
                        spkt.kind = 'pair'
                        spkt.pair = SurveyPair(src=src, dst=dst, dist_m=dist_m, ok=ok)
                except (IndexError, ValueError):
                    pass
            return spkt

        return spkt   # kind remains None — not a survey line
