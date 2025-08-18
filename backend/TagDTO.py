from datetime import datetime
from dataclasses import dataclass

@dataclass
class TagDTO:
    date: datetime
    anc0: float
    anc3: float
    anc4: float
    anc6: float
    fin_pos: str