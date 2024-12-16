import sqlite3
from pathlib import Path


def create_database(conn):
    """ Create a new SQLite Database with default parameters

    Args:
        conn:
    """
    c = conn.cursor()

    # Create Defaults Table
    c.execute("CREATE TABLE `defaults` ( `live` INTEGER, `ascent_rate` INTEGER, `descent_rate` INTEGER, "
              "`timeout_0` INTEGER, `timeout_1` INTEGER, `timeout_2` INTEGER , `timeout_3` INTEGER, "
              "`ramp_rate` INTEGER )")
    # Create cycles table
    c.execute("CREATE TABLE `cycles` ( `id` INTEGER PRIMARY KEY, "
              "`date` DATETIME DEFAULT (datetime('now', 'localtime')), `lat` REAL, `lon` REAL )")
    # Create cycles data table
    c.execute("CREATE TABLE `cycle_data` ( `id` INTEGER PRIMARY KEY, `file` TEXT )")
    # Create properties data table
    c.execute("CREATE TABLE `cycle_properties` ( `id` INTEGER PRIMARY KEY, `ascent_rate` INTEGER, "
              "`descent_rate` INTEGER, `timeout_0` INTEGER, `timeout_1` INTEGER, `timeout_2` INTEGER, "
              "`timeout_3` INTEGER, `ramp_rate` INTEGER )")
    # Create log entries table
    c.execute("CREATE TABLE `log` ( `date` DATETIME DEFAULT (datetime('now', 'localtime')), `entry` TEXT )")

    # Default Values
    c.execute("INSERT INTO defaults (live, ascent_rate, descent_rate, timeout_0, timeout_1, timeout_2, timeout_3, "
              "ramp_rate) VALUES (?, ?, ?, ?, ?, ?, ?, ?)", (0, 50, 50, 5000, 100, 5000, 100, 3))
    # Live Values
    c.execute("INSERT INTO defaults (live, ascent_rate, descent_rate, timeout_0, timeout_1, timeout_2, timeout_3, "
              "ramp_rate) VALUES (?, ?, ?, ?, ?, ?, ?, ?)", (1, 50, 50, 5000, 100, 5000, 100, 3))

    conn.commit()


if __name__ == '__main__':
    db = Path('assets/logging.db')
    if not db.exists():
        conn = sqlite3.connect(str(db))
        create_database(conn)
        conn.close()
