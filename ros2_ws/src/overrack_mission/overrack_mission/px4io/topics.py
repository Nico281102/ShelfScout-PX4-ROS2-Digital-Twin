"""Topic helpers to add optional vehicle namespaces."""

from __future__ import annotations

from typing import Optional


def namespaced(topic: str, *, namespace: Optional[str] = None, absolute: bool = True) -> str:
    """Return a topic with an optional namespace prefix.

    - Removes leading slashes from ``topic``.
    - If ``namespace`` is provided, prefixes ``<namespace>/`` (namespace cleaned of leading/trailing slashes).
    - When ``absolute`` is True, prepends a leading slash to the final result.
    """

    clean_topic = (topic or "").lstrip("/")
    if not clean_topic:
        return "/" if absolute else ""

    if namespace:
        ns = namespace.strip("/")
        clean_topic = f"{ns}/{clean_topic}"

    return f"/{clean_topic}" if absolute else clean_topic
