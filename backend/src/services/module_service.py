from sqlalchemy.orm import Session
from typing import List
from models.module import Module
from models.chapter import Chapter
from pydantic import BaseModel

class ModuleCreate(BaseModel):
    name: str
    description: str
    order_index: int
    learning_objectives: List[str]
    prerequisites: List[str] = []

class ModuleResponse(BaseModel):
    id: str
    name: str
    description: str
    order_index: int
    learning_objectives: List[str]
    prerequisites: List[str]
    created_at: str

class ModuleService:
    """
    Service class for handling module-related operations
    """
    
    def __init__(self, db: Session):
        self.db = db
    
    def create_module(self, module_data: ModuleCreate) -> Module:
        """
        Create a new textbook module
        """
        from uuid import uuid4
        module = Module(
            id=str(uuid4()),
            name=module_data.name,
            description=module_data.description,
            order_index=module_data.order_index,
            learning_objectives=module_data.learning_objectives,
            prerequisites=module_data.prerequisites
        )
        self.db.add(module)
        self.db.commit()
        self.db.refresh(module)
        return module
    
    def get_module_by_id(self, module_id: str) -> Module:
        """
        Retrieve a module by its ID
        """
        return self.db.query(Module).filter(Module.id == module_id).first()
    
    def get_all_modules(self) -> List[Module]:
        """
        Retrieve all textbook modules
        """
        return self.db.query(Module).order_by(Module.order_index).all()
    
    def update_module(self, module_id: str, module_data: ModuleCreate) -> Module:
        """
        Update an existing module
        """
        module = self.get_module_by_id(module_id)
        if module:
            for key, value in module_data.dict().items():
                setattr(module, key, value)
            self.db.commit()
            self.db.refresh(module)
        return module
    
    def delete_module(self, module_id: str) -> bool:
        """
        Delete a module by its ID
        """
        module = self.get_module_by_id(module_id)
        if module:
            # Delete associated chapters first
            chapters = self.db.query(Chapter).filter(Chapter.module_id == module_id).all()
            for chapter in chapters:
                self.db.delete(chapter)
            
            # Now delete the module
            self.db.delete(module)
            self.db.commit()
            return True
        return False
    
    def get_module_with_chapters(self, module_id: str) -> Module:
        """
        Retrieve a module with its associated chapters
        """
        module = self.get_module_by_id(module_id)
        if module:
            chapters = self.db.query(Chapter).filter(
                Chapter.module_id == module_id
            ).order_by(Chapter.order_index).all()
            module.chapters = chapters
        return module

# Example usage
if __name__ == "__main__":
    print("Module service created")