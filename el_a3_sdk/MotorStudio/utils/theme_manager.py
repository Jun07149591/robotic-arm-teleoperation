"""主题与语言管理器（单例）"""

from PyQt6.QtCore import QObject, pyqtSignal


class ThemeManager(QObject):
    """全局主题 / 语言管理，各面板订阅信号后自行刷新。"""

    theme_changed = pyqtSignal(str)     # "dark" | "light"
    language_changed = pyqtSignal(str)  # "zh" | "en"

    _instance: "ThemeManager | None" = None

    def __init__(self):
        super().__init__()
        self._theme = "dark"
        self._language = "zh"

    @classmethod
    def instance(cls) -> "ThemeManager":
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    @property
    def theme(self) -> str:
        return self._theme

    @property
    def language(self) -> str:
        return self._language

    def set_theme(self, theme: str):
        if theme == self._theme:
            return
        self._theme = theme
        self.theme_changed.emit(theme)

    def set_language(self, lang: str):
        if lang == self._language:
            return
        self._language = lang
        self.language_changed.emit(lang)

    def toggle_theme(self):
        self.set_theme("light" if self._theme == "dark" else "dark")

    def toggle_language(self):
        self.set_language("en" if self._language == "zh" else "zh")

    @property
    def is_dark(self) -> bool:
        return self._theme == "dark"
