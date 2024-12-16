library("rjson")
library("plotly")
library("changepoint")

setwd("/home/kieren/code/surface-automation/binmaster/scripts/visualiser/logs")

tension <- as.data.frame(fromJSON(file = "121.json"))$tension
tension_sub <- tension[2000:which.min(tension)]

tension_shift <- cpt.meanvar(tension_sub, penalty = "Asymptotic", pen.value = 0.5)
plot(tension_shift, cpt.col = 'blue')
print(tension_shift)

print(cpts(tension_shift))

#fig <- plot_ly(y = ~tension, type = 'scatter', mode = 'line')
#
#htmlwidgets::saveWidget(fig, "temp.html")