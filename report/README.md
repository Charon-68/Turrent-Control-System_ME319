# Report

`main.tex` is the IEEE-style project report (IEEEtran, conference mode).

## Build

```
cd report && pdflatex main && pdflatex main
```

Run `pdflatex` twice so cross-references resolve. No BibTeX run is needed (the references are a `thebibliography` block).
Packages used: `amsmath`, `amssymb`, `graphicx`, `booktabs`, `cite`, `url` (all in TeX Live and Overleaf).

## Inputs

- `generated/`: `numbers.tex` (macros for every number in the report) and the `*_table.tex` tables. They are written by
  `studies/tc_export_numbers.m` from the files in `outputs/`; never edit them by hand. Regenerate them from the repository root with

  ```
  make export
  ```

- Figures: the six PDFs in `../plots/` that `main.tex` includes (`report_schematic`, `report_blockdiagram`, `design_analysis`,
  `report_estimation`, `report_scenarios`, `report_final_eval`). `make report-figures` regenerates the report figures; the
  commands for each are in `outputs/README.md`. `main.tex` finds them through `\graphicspath{{../plots/}{plots/}}`, so it compiles both
  in this repository and in the flat Overleaf bundle.

## Overleaf

From the repository root,

```
make report-zip
```

writes `build/overleaf.zip` with `main.tex`, `generated/*.tex` and a `plots/` folder holding only the PDFs that `main.tex` includes.
Upload the zip as a new Overleaf project; main file `main.tex`, compiler pdfLaTeX.
