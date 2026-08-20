"""Persistence for sigmon: MongoDB, with an explicit JSONL fallback.

The fallback exists because a monitoring run that silently discards its
observations when the database is down is worse than one that refuses to start.
If Mongo is unreachable the run continues and every record is appended to a
JSONL file instead, and that substitution is announced loudly at startup and
again in the summary -- never inferred from an empty collection later.
"""
import datetime
import json
import os
import sys


def utcnow():
    return datetime.datetime.now(datetime.timezone.utc)


class Store:
    """Writes observations to Mongo if it is reachable, else to JSONL."""

    def __init__(self, uri, db_name, fallback_path, timeout_ms=3000,
                 quiet=False):
        self.db_name = db_name
        self.fallback_path = fallback_path
        self.mongo = None
        self.client = None
        self.reason = None
        self._fallback_fh = None
        self.counts = {"observations": 0, "signals": 0, "runs": 0}

        try:
            import pymongo
        except ImportError:
            self.reason = "pymongo is not installed (pip install --user pymongo)"
        else:
            try:
                self.client = pymongo.MongoClient(
                    uri, serverSelectionTimeoutMS=timeout_ms)
                self.client.admin.command("ping")
                self.mongo = self.client[db_name]
                self._ensure_indexes()
            except Exception as e:                          # noqa: BLE001
                self.reason = f"{type(e).__name__}: {str(e)[:120]}"
                self.client = None
                self.mongo = None

        if not quiet:
            if self.mongo is not None:
                print(f"[store] MongoDB {uri} db={db_name}")
            else:
                print(f"[store] MongoDB unavailable -- {self.reason}")
                print(f"[store] FALLING BACK to JSONL: {self.fallback_path}")
                print("[store] records are NOT in the database for this run")

    def _ensure_indexes(self):
        # Queries are always "this signal, over time" or "what was around at
        # time T", so index for both.
        self.mongo.observations.create_index([("freq_hz", 1), ("ts", -1)])
        self.mongo.observations.create_index([("run_id", 1)])
        self.mongo.signals.create_index([("freq_hz", 1)], unique=False)
        self.mongo.signals.create_index([("last_seen", -1)])

    @property
    def using_mongo(self):
        return self.mongo is not None

    def _jsonl(self, collection, doc):
        if self._fallback_fh is None:
            os.makedirs(os.path.dirname(os.path.abspath(self.fallback_path)) or ".",
                        exist_ok=True)
            self._fallback_fh = open(self.fallback_path, "a", buffering=1)
        rec = dict(doc)
        rec["_collection"] = collection
        self._fallback_fh.write(
            json.dumps(rec, default=str, separators=(",", ":")) + "\n")

    def insert_run(self, doc):
        self.counts["runs"] += 1
        if self.using_mongo:
            return self.mongo.runs.insert_one(doc).inserted_id
        self._jsonl("runs", doc)
        return doc.get("run_id")

    def insert_observation(self, doc):
        self.counts["observations"] += 1
        if self.using_mongo:
            self.mongo.observations.insert_one(doc)
        else:
            self._jsonl("observations", doc)

    def upsert_signal(self, freq_hz, update):
        """One document per emitter, carrying its running bearing statistics."""
        self.counts["signals"] += 1
        if self.using_mongo:
            self.mongo.signals.update_one(
                {"freq_hz": freq_hz},
                {"$set": update, "$setOnInsert": {"first_seen": utcnow()}},
                upsert=True)
        else:
            d = dict(update)
            d["freq_hz"] = freq_hz
            self._jsonl("signals", d)

    def recent_bearings(self, freq_hz, limit=200):
        """Past bearings for one emitter, newest first -- used for stability."""
        if not self.using_mongo:
            return []
        cur = self.mongo.observations.find(
            {"freq_hz": freq_hz, "bearing_deg": {"$ne": None}},
            {"bearing_deg": 1, "_id": 0}).sort("ts", -1).limit(limit)
        return [d["bearing_deg"] for d in cur]

    def close(self):
        if self._fallback_fh:
            self._fallback_fh.close()
            self._fallback_fh = None
        if self.client:
            self.client.close()

    def summary(self):
        where = (f"MongoDB db={self.db_name}" if self.using_mongo
                 else f"JSONL {self.fallback_path} (MongoDB was unavailable)")
        return (f"wrote {self.counts['observations']} observations, "
                f"{self.counts['signals']} signal updates -> {where}")
