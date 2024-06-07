import pandas as pd
from matplotlib import ticker as tck


file = "data/forest.csv"
df = pd.read_csv(
    file,
    dtype=float,
    index_col=0,
    names=["Time", "Biomass", "Growth rate", "Burned area"],
)

axes = df.plot(
    subplots=True,
    layout=(3, 1),
    legend=False,
    grid=True,
    title=list(df.columns.values),
)

for ax in axes.flat:
    ax.xaxis.set_minor_locator(tck.AutoMinorLocator())
    ax.yaxis.set_minor_locator(tck.AutoMinorLocator())
    ax.grid(visible=True, which="both")

fig = axes.flat[0].get_figure()
fig.tight_layout(h_pad=1)
fig.savefig("images/graph.png")
