# Project Overview

# Project Overview

## Biological Obsidian System

This project is a flat, link-driven system for representing biological evolution in Obsidian.
It is not a taxonomy database, but a selective evolutionary graph that grows only when needed.

## Core Principle

Structure lives in links, not in files, folders, or tags.

- Each note represents one biological unit (clade or group).
- Notes connect through a single Up link.
- Navigation is handled through fixed MOC hubs.
- Tags are optional metadata only, not required for structure.

## System Layers

### 1. Truth Layer (Structure)

This is the evolutionary graph.

- Defined only by Up links.
- Each note has exactly one parent, except root.

Example:

```text
Homo sapiens → Homo → Hominidae → Primates → Mammalia → ...
```

### 2. Navigation Layer (MOCs)

There are exactly 7 fixed MOCs:

- Domains MOC
- Kingdoms MOC
- Phyla MOC
- Classes MOC
- Orders MOC
- Families MOC
- Genera MOC

Each note belongs to exactly one closest MOC.
If biology does not fit perfectly, assign the closest meaningful level.

### 3. Detail Layer (Optional)

- Species-level notes exist only when explicitly generated.
- Deep detail is controlled by the Python script.
- Not part of default graph exploration.

## CSV Manifest and Cache

The editable CSV is the source manifest for note generation.
It stays intentionally sparse so the script can resolve and enrich taxa from the API when needed.

Resolved taxonomy can be written to a separate cache CSV for repeat runs.
That cache is only an optimization and can always be regenerated from the manifest.

### CSV Format (Minimal Design)

Each row represents one note.
Only three fields are required:

```text
name,up,moc,tags
```

### Column Definitions

#### name

The biological entity name.

Used as note title and filename.

#### up

The parent node (Up link).

Defines evolutionary structure.

Example:

```text
Homo sapiens → Homo
```

Root nodes (e.g. Life) leave this empty.

#### moc

The navigation level assignment.

Must be one of the 7 fixed MOCs:

- Domains MOC
- Kingdoms MOC
- Phyla MOC
- Classes MOC
- Orders MOC
- Families MOC
- Genera MOC

## Python Generator Responsibilities

The script must:

- Read each CSV row.
- Resolve taxonomy from the API when a row is incomplete.
- Check if file already exists.
- If yes, do nothing.
- If missing, create note.
- Insert:
  - name
  - Up link
  - MOC link
  - Optional tags
- Never modify existing notes.
- Keep output flat, with no folders.
- Write an optional cache CSV with resolved API data for faster reruns.

## Note Template (Generated)

```md
---
tags:
  - tag1
  - tag2
---

Up :: [[{up}]]
MOC :: [[{moc}]]

# **`=this.file.name`**
```

## Design Philosophy

This system prioritizes:

- Minimal input
- Maximum structural clarity
- Zero redundancy
- Safe regeneration, with no overwrites
- Evolutionary correctness over taxonomic completeness

## One-Line Summary

A biological knowledge graph where a minimal CSV (name, up, moc) generates a flat Obsidian vault, with all structure derived from links and all navigation handled through fixed MOC levels.
