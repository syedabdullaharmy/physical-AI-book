import uuid
from typing import Optional, List

from fastapi_users import schemas

class UserRead(schemas.BaseUser[uuid.UUID]):
    first_name: Optional[str]
    last_name: Optional[str]
    bookmarks: List[str] = []
    finished_chapters: List[str] = []
    software_background: Optional[str]
    hardware_background: Optional[str]

class UserCreate(schemas.BaseUserCreate):
    first_name: Optional[str]
    last_name: Optional[str]
    software_background: Optional[str]
    hardware_background: Optional[str]

class UserUpdate(schemas.BaseUserUpdate):
    first_name: Optional[str]
    last_name: Optional[str]
    bookmarks: Optional[List[str]]
    finished_chapters: Optional[List[str]]
    software_background: Optional[str]
    hardware_background: Optional[str]
