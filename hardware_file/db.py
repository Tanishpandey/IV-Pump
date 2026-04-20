#!/usr/bin/env python3
"""
db.py — Shared database module
Works on both the Pi (pi_agent.py) and the laptop (plotter_gui.py).

Uses SQLite — no server needed.
On the laptop, the DB stores login accounts and button press logs locally.
"""

import sqlite3
import hashlib
import os
from datetime import datetime
from typing import Optional

DB_PATH = os.path.join(os.path.dirname(__file__), "plotter.db")


# ============================================================================
# Connection helper
# ============================================================================

def _connect() -> sqlite3.Connection:
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA journal_mode=WAL")   # safe for concurrent access
    return conn


# ============================================================================
# Schema
# ============================================================================

def init_db():
    """Create tables and seed demo users if the DB is new."""
    with _connect() as conn:
        conn.executescript("""
            CREATE TABLE IF NOT EXISTS users (
                id           INTEGER PRIMARY KEY AUTOINCREMENT,
                username     TEXT    NOT NULL UNIQUE,
                display_name TEXT    NOT NULL,
                password_hash TEXT   NOT NULL,
                role         TEXT    NOT NULL DEFAULT 'operator',
                created_at   TEXT    NOT NULL DEFAULT (datetime('now'))
            );

            CREATE TABLE IF NOT EXISTS button_log (
                id           INTEGER PRIMARY KEY AUTOINCREMENT,
                user_id      INTEGER NOT NULL,
                username     TEXT    NOT NULL,
                pressed_at   TEXT    NOT NULL DEFAULT (datetime('now')),
                target_x_mm  REAL    NOT NULL,
                target_y_mm  REAL    NOT NULL,
                servo_used   INTEGER,
                press_dist   REAL,
                status       TEXT    NOT NULL DEFAULT 'success',
                note         TEXT
            );
        """)

        # Seed demo accounts if table is empty
        row = conn.execute("SELECT COUNT(*) FROM users").fetchone()[0]
        if row == 0:
            _seed_users(conn)
            print("[db] Demo users created.")


def _hash(password: str) -> str:
    return hashlib.sha256(password.encode()).hexdigest()


def _seed_users(conn: sqlite3.Connection):
    users = [
        ("admin",    "Administrator", _hash("admin123"),    "admin"),
        ("operator", "Operator",      _hash("operator456"), "operator"),
    ]
    conn.executemany(
        "INSERT INTO users (username, display_name, password_hash, role) VALUES (?,?,?,?)",
        users
    )


# ============================================================================
# Auth
# ============================================================================

def authenticate(username: str, password: str) -> Optional[sqlite3.Row]:
    """Return the user row if credentials match, else None."""
    with _connect() as conn:
        row = conn.execute(
            "SELECT * FROM users WHERE username = ?", (username,)
        ).fetchone()
        if row and row["password_hash"] == _hash(password):
            return row
        return None


def get_user(username: str) -> Optional[sqlite3.Row]:
    with _connect() as conn:
        return conn.execute(
            "SELECT * FROM users WHERE username = ?", (username,)
        ).fetchone()


def create_user(username: str, display_name: str,
                password: str, role: str = "operator") -> bool:
    """Create a new user. Returns False if username already exists."""
    try:
        with _connect() as conn:
            conn.execute(
                "INSERT INTO users (username, display_name, password_hash, role) VALUES (?,?,?,?)",
                (username, display_name, _hash(password), role)
            )
        return True
    except sqlite3.IntegrityError:
        return False


def change_password(username: str, new_password: str) -> bool:
    with _connect() as conn:
        cur = conn.execute(
            "UPDATE users SET password_hash = ? WHERE username = ?",
            (_hash(new_password), username)
        )
        return cur.rowcount > 0


# ============================================================================
# Button log
# ============================================================================

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
    """Insert a log entry and return its row id."""
    with _connect() as conn:
        cur = conn.execute(
            """INSERT INTO button_log
               (user_id, username, pressed_at, target_x_mm, target_y_mm,
                servo_used, press_dist, status, note)
               VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)""",
            (user_id, username, datetime.now().isoformat(sep=" ", timespec="seconds"),
             target_x_mm, target_y_mm, servo_used, press_dist, status, note)
        )
        return cur.lastrowid


def get_log(limit: int = 200):
    """Return the most recent log entries (newest first)."""
    with _connect() as conn:
        rows = conn.execute(
            """SELECT * FROM button_log
               ORDER BY pressed_at DESC
               LIMIT ?""",
            (limit,)
        ).fetchall()
        return [dict(r) for r in rows]


def get_log_for_user(username: str, limit: int = 200):
    """Return log entries for a specific user (newest first)."""
    with _connect() as conn:
        rows = conn.execute(
            """SELECT * FROM button_log
               WHERE username = ?
               ORDER BY pressed_at DESC
               LIMIT ?""",
            (username, limit)
        ).fetchall()
        return [dict(r) for r in rows]


def get_log_stats():
    """Return summary statistics."""
    with _connect() as conn:
        total   = conn.execute("SELECT COUNT(*) FROM button_log").fetchone()[0]
        success = conn.execute(
            "SELECT COUNT(*) FROM button_log WHERE status='success'").fetchone()[0]
        error   = conn.execute(
            "SELECT COUNT(*) FROM button_log WHERE status='error'").fetchone()[0]
        return {"total": total, "success": success, "error": error}


# ============================================================================
# CLI helper — run `python3 db.py` to inspect the DB
# ============================================================================

if __name__ == "__main__":
    init_db()
    print("=== Users ===")
    with _connect() as conn:
        for row in conn.execute("SELECT id, username, display_name, role FROM users"):
            print(dict(row))
    print("\n=== Recent log (last 10) ===")
    for entry in get_log(10):
        print(entry)
    print(f"\n=== Stats ===\n{get_log_stats()}")
