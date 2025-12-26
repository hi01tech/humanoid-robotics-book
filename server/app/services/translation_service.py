import google.generativeai as genai
from ..core.config import settings

class TranslationService:
    def __init__(self):
        # Configure the generative model for translation
        genai.configure(api_key=settings.google_api_key)
        self.model = genai.GenerativeModel('gemini-2.5-flash')
        self._cache = {} # Add cache

    def translate_text(self, text: str, target_language: str = "ur") -> str:
        """Translates text into the target language using the Gemini API."""

        # Check cache first
        cache_key = (text, target_language)
        if cache_key in self._cache:
            return self._cache[cache_key]

        # A simple mapping from language codes to full language names for the prompt
        lang_map = {
            "ur": "Urdu",
            "en": "English",
        }
        target_lang_name = lang_map.get(target_language, target_language)

        prompt = f"Translate the following English text to {target_lang_name}. Provide the translation only in {target_lang_name} script. Do not include Roman {target_lang_name} or any transliteration. \n\nEnglish Text: {text}"

        try:
            response = self.model.generate_content(prompt)
            translated_text = response.text
            # Store in cache
            self._cache[cache_key] = translated_text
            return translated_text
        except Exception as e:
            print(f"Error during Gemini API translation: {e}")
            return f"[Translation Error: {e}]"