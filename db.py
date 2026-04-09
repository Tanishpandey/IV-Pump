#!/usr/bin/env python3
"""
db.py — SQLite database layer for XY Plotter

Tables:
  users       — login credentials (hashed passwords)
  button_log  — record of every button press with user + timestamp
"""

import sqlite3
import hashlib
import os
from datetime import datetime
from typing import Optional

DB_PATH = os.path.join(os.path.dirname(__file__), "plotter.db")


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _connect() -> sqlite3.Connection:
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row          # dict-like rows
    conn.execute("PRAGMA foreign_keys = ON")
    return conn


def _hash(password: str) -> str:
    """SHA-256 hash – good enough for a local-only tool."""
    return hashlib.sha256(password.encode()).hexdigest()


# ---------------------------------------------------------------------------
# Schema
# ---------------------------------------------------------------------------

def init_db():
    """Create tables and seed dummy users if the database is new."""
    with _connect() as conn:
        conn.executescript("""
            CREATE TABLE IF NOT EXISTS users (
                id            INTEGER PRIMARY KEY AUTOINCREMENT,
                username      TEXT    NOT NULL UNIQUE,
                password_hash TEXT    NOT NULL,
                display_name  TEXT    NOT NULL,
                created_at    TEXT    NOT NULL DEFAULT (datetime('now'))
            );

            CREATE TABLE IF NOT EXISTS button_log (
                id          INTEGER PRIMARY KEY AUTOINCREMENT,
                user_id     INTEGER NOT NULL REFERENCES users(id),
                username    TEXT    NOT NULL,
                target_x_mm REAL    NOT NULL,
                target_y_mm REAL    NOT NULL,
                servo_used  INTEGER,
                press_dist  REAL,
                status      TEXT    NOT NULL DEFAULT 'success',
                note        TEXT,
                pressed_at  TEXT    NOT NULL DEFAULT (datetime('now', 'localtime'))
            );

            CREATE INDEX IF NOT EXISTS idx_log_user ON button_log(user_id);
            CREATE INDEX IF NOT EXISTS idx_log_time ON button_log(pressed_at);
        """)

        # Seed two dummy accounts only if the table is empty
        existing = conn.execute("SELECT COUNT(*) FROM users").fetchone()[0]
        if existing == 0:
            conn.executemany(
                "INSERT INTO users (username, password_hash, display_name) VALUES (?,?,?)",
                [
                    ("nurse1",    _hash("nurse123"),    "nurse1"),
                    ("nurse2", _hash("nurse123"), "nurse2"),
                ]
            )
            print("[db] Seeded dummy users: nurse1 / nurse2")


# ---------------------------------------------------------------------------
# Auth
# ---------------------------------------------------------------------------

def authenticate(username: str, password: str) -> Optional[sqlite3.Row]:
    """
    Return the user row if credentials match, else None.

    Usage:
        user = authenticate("admin", "admin123")
        if user:
            print(user["display_name"])
    """
    with _connect() as conn:
        row = conn.execute(
            "SELECT * FROM users WHERE username = ?", (username,)
        ).fetchone()
    if row and row["password_hash"] == _hash(password):
        return row
    return None


# ---------------------------------------------------------------------------
# Button logging
# ---------------------------------------------------------------------------

def log_button_press(
    user_id: int,
    username: str,
    target_x_mm: float,
    target_y_mm: float,
    servo_used: Optional[int] = None,
    press_dist: Optional[float] = None,
    status: str = "success",
    note: Optional[str] = None,
) -> int:
    """
    Insert a button-press record.  Returns the new row id.
    """
    with _connect() as conn:
        cur = conn.execute(
            """
            INSERT INTO button_log
                (user_id, username, target_x_mm, target_y_mm,
                 servo_used, press_dist, status, note)
            VALUES (?,?,?,?,?,?,?,?)
            """,
            (user_id, username, target_x_mm, target_y_mm,
             servo_used, press_dist, status, note),
        )
        return cur.lastrowid


def get_log(limit: int = 200) -> list:
    """Return the most recent `limit` log entries, newest first."""
    with _connect() as conn:
        rows = conn.execute(
            """
            SELECT bl.id, bl.username, bl.target_x_mm, bl.target_y_mm,
                   bl.servo_used, bl.press_dist, bl.status, bl.note,
                   bl.pressed_at
            FROM button_log bl
            ORDER BY bl.pressed_at DESC
            LIMIT ?
            """,
            (limit,),
        ).fetchall()
    return [dict(r) for r in rows]


def get_log_for_user(username: str, limit: int = 100) -> list:
    """Return log entries for one specific user."""
    with _connect() as conn:
        rows = conn.execute(
            """
            SELECT * FROM button_log
            WHERE username = ?
            ORDER BY pressed_at DESC
            LIMIT ?
            """,
            (username, limit),
        ).fetchall()
    return [dict(r) for r in rows]


# ---------------------------------------------------------------------------
# Convenience: run directly to inspect the database
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    init_db()
    print("\n=== Users ===")
    with _connect() as c:
        for row in c.execute("SELECT id, username, display_name, created_at FROM users"):
            print(dict(row))
    print("\n=== Last 10 button presses ===")
    for entry in get_log(10):
        print(entry)
