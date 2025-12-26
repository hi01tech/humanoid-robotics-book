from fastapi import APIRouter
from pydantic import BaseModel

from ..services.translation_service import TranslationService

router = APIRouter()
translation_service = TranslationService()

class TranslationRequest(BaseModel):
    text: str
    target_language: str = 'ur'

class TranslationResponse(BaseModel):
    translated_text: str

@router.post("/translate-text", response_model=TranslationResponse)
async def translate_text(request: TranslationRequest):
    """
    Accepts text and a target language, returns the translated text.
    """
    translated = translation_service.translate_text(
        text=request.text,
        target_language=request.target_language
    )
    return TranslationResponse(translated_text=translated)
