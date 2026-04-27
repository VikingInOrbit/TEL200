# Biological Obsidian Note System

## Purpose

This project defines a flat, tag-based Obsidian system for biological knowledge.
It is inspired by evolutionary relationships rather than strict textbook taxonomy.
The goal is to build a living graph of life that grows only when needed.

## Core Idea

- Each note represents one biological group, usually a clade or other meaningful unit.
- Species are excluded by default.
- Deeper detail is added only when curiosity or use case requires it.
- Structure comes from links and tags, not folders.
- Navigation comes from tags, links, and curated MOCs.

## Hard Rules

- No folders for biological structure.
- Use tags instead of folders.
- Do not mirror taxonomy in the filesystem.
- Do not create subfolders by rank, branch, or lineage.
- Do not overwrite existing notes.
- Create notes only when they are missing and allowed to be created.
- Keep the system expandable, not exhaustive.

## Organization Model

The system has three layers:

### 1. Truth Layer

This is the evolutionary graph itself.
It is built with note-to-note links.
Example:

- Protostomia links up to Bilateria
- Bilateria links up to Metazoa

### 2. Navigation Layer

This is the MOC and tag layer.
It helps you browse the graph without defining biology itself.
Use tags like this:

- #domains
- #domains/kingdoms
- #domains/kingdoms/phyla
- #domains/kingdoms/phyla/classes
- #domains/kingdoms/phyla/classes/orders
- #domains/kingdoms/phyla/classes/orders/families
- #domains/kingdoms/phyla/classes/orders/families/genera

### 3. Detail Layer

This is optional deep detail.
It may include genera and species, but only when explicitly needed.

## Tag Strategy

Tags replace folders.
Tags should be short, consistent, and hierarchical.

### Sorting Tags

- #bio/life
- #bio/life/eukaryota
- #bio/life/eukaryota/metazoa
- #bio/life/eukaryota/metazoa/bilateria
- #bio/life/eukaryota/metazoa/bilateria/protostomia
- #bio/life/eukaryota/metazoa/bilateria/protostomia/arthropoda
- #bio/life/eukaryota/metazoa/bilateria/protostomia/insecta
- #bio/life/eukaryota/metazoa/bilateria/protostomia/insecta/formicidae

### Tag Rules

- Use tags to show membership and scope.
- Use tags to classify note type, such as:
  - #type/clade
  - #type/moc
  - #type/species
  - #type/deep-detail
- Use tags for exploration stages, such as:
  - #depth/backbone
  - #depth/branch
  - #depth/optional
- Keep the tag set small and predictable.
- Do not use tags as a replacement for note content.

## Link Strategy

Each note should support two important link roles:

### Up Link

This points to the evolutionary parent.
It is the structural link.
Example:

- Protostomia -> Bilateria
- Bilateria -> Metazoa

### MOC Link

This points to a navigation hub.
It is the browsing link.
Example:

- Arthropoda -> MOC: Phyla
- Insecta -> MOC: Classes
- Formicidae -> MOC: Families

## Note Meaning

Each note should answer a simple question:
What biological group does this represent, and what is its parent?

A note is not supposed to hold a full encyclopedia entry by default.
It is a node in a living graph.

## Depth Policy

The system grows only as far as needed.

### Backbone Notes

These are the main structural notes.
Examples:

- Life
- Eukaryota
- Metazoa
- Bilateria
- Protostomia
- Arthropoda
- Insecta
- Formicidae

### Optional Deep Notes

These appear only if needed.
Examples:

- Genus notes
- Species notes
- Special subgroups for research or curiosity

## Project Manifest

A future Python script should read a CSV file and generate notes from it.
The CSV is the project manifest.
It should contain enough information to create notes safely without overwriting existing files.

### Recommended CSV Columns

- note_name
- note_type
- parent_note
- moc_name
- tags
- file_name
- create_if_missing
- include_species
- depth_level
- notes

### Column Meaning

- note_name: biological group name
- note_type: clade, group, MOC, species, or deep-detail marker
- parent_note: the Up link target
- moc_name: the navigation hub for the note
- tags: tag list for classification and browsing
- file_name: markdown filename
- create_if_missing: whether the script may create the file
- include_species: whether deep species detail is allowed
- depth_level: how deep the note is in the graph
- notes: short hints, exceptions, or generation notes

## Generation Rules

The future script should follow these rules:

1. Read one CSV row per note.
2. Check whether the target file already exists.
3. If it exists, do nothing.
4. If it does not exist and create_if_missing is true, create the note.
5. If create_if_missing is false, skip creation.
6. Never rewrite existing notes.
7. Never force-create parent notes unless their own CSV rows allow it.
8. Always keep the filesystem flat.
9. Use the note content for links and tags, not folders.
10. Leave room for later expansion.

## Suggested Note Template

A generated note can follow a minimal standard form:

- Title
- Tags
- Up link
- MOC link
- Short body placeholder

Example skeleton:

```markdown
# Arthropoda

Tags: #bio/arthropoda #type/clade #depth/backbone

Up: [[Bilateria]]
MOC: [[Phyla MOC]]

## Notes

Short placeholder text.
```

## Example Backbone Path

A basic evolutionary chain might look like this:

- Life
- Eukaryota
- Metazoa
- Bilateria
- Protostomia
- Arthropoda
- Insecta
- Formicidae

This is not a complete taxonomy dump.
It is a selective graph backbone.

## Example Tag-Only Navigation

Instead of folders, use tags to browse:

- #bio/metazoa for all metazoan notes
- #bio/arthropoda for arthropod notes
- #moc/phyla for phylum-level navigation
- #depth/optional for curiosity-driven entries

## File Layout

The project should stay flat.

Recommended layout:

- one root-level `bio.md` reference file
- one flat notes area for generated notes
- no nested biological folders

If a container folder is used at all, it should only be a single flat storage location.
It should not encode biological meaning.

## What This System Is

- A phylogenetic tree in note form
- A navigation map through MOCs and tags
- A selective exploration system
- A living knowledge graph

## What This System Is Not

- Not a full taxonomy dump
- Not a folder hierarchy
- Not a rigid database export
- Not a rewrite of existing notes
- Not a species encyclopedia by default

## One-Line Summary

A selectively detailed evolutionary graph of life in Obsidian, where tags replace folders, links handle structure, and depth is added only when needed.
