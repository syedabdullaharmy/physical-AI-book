from fastapi_users.db import SQLAlchemyBaseUserTableUUID
from sqlalchemy.orm import Mapped, mapped_column
from sqlalchemy.types import JSON
from typing import List, Optional
from app.db.database import Base

class User(SQLAlchemyBaseUserTableUUID, Base):
    __tablename__ = "user"

    first_name: Mapped[Optional[str]] = mapped_column(nullable=True)
    last_name: Mapped[Optional[str]] = mapped_column(nullable=True)
    bookmarks: Mapped[List[str]] = mapped_column(JSON, default=list)
    finished_chapters: Mapped[List[str]] = mapped_column(JSON, default=list)
    software_background: Mapped[Optional[str]] = mapped_column(nullable=True)
    hardware_background: Mapped[Optional[str]] = mapped_column(nullable=True)
