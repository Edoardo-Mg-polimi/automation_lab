in this folder save the files for the latex report
Magnetic Levitation report, LaTeX structure


How to compile locally:

pdflatex report.tex
pdflatex report.tex

How to use on Overleaf:

1. Upload the whole folder content.
2. Set report.tex as main file.
3. Compile with pdfLaTeX.

Where to add figures:

You can create a figures folder and include images with:
\includegraphics[width=0.85\textwidth]{figures/name_of_the_figure.pdf}

Recommended workflow:

1. Keep report.tex only for packages, title page, table of contents, section inputs and bibliography.
2. Write all technical content inside the files in sections.
3. Use one figure per result, with a clear caption and label.
4. For every controller, report design, simulation, real validation and final comments.
5. Keep the final report within the required page limit.
