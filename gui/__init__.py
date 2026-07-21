"""GUI module for Arjuna GCS."""

__all__ = ["ArjunaShell", "MainWindow"]


def __getattr__(name: str):
    if name == "ArjunaShell":
        from gui.arjuna_shell import ArjunaShell

        return ArjunaShell
    if name == "MainWindow":
        from gui.main_window import MainWindow

        return MainWindow
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
