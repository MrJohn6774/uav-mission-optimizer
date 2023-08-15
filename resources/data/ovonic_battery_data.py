import pandas as pd
from sklearn.linear_model import LinearRegression, Ridge

# Defining the path to the uploaded Excel file
battery_data_path = 'ovonic_battery_data.xlsx'

# Loading the data from the Excel file
battery_data = pd.read_excel(battery_data_path)

# Displaying the first few rows to understand the structure of the data
print(battery_data.head())

# Extracting the number of cells and number of packs from the "Config" column
battery_data["Number of Cells"] = battery_data["Config"].apply(lambda x: int(x.upper().split('S')[0]))
battery_data["Number of Packs"] = battery_data["Config"].apply(lambda x: int(x.upper().split('P')[0][-1]))

# Calculating the maximum discharge current in Ah (Capacity * Discharge rating / 1000)
battery_data["Max Discharge Current (A)"] = battery_data["Capacity (mAh)"] * battery_data["Discharge (C)"] / 1000

# Dividing the weight by the number of packs to get the weight per pack
battery_data["Weight per Pack (g)"] = battery_data["Weight (g)"] / battery_data["Number of Packs"]

# Creating an interaction term between Capacity and Max Discharge Current
battery_data["Capacity-Discharge Interaction"] = battery_data["Capacity (mAh)"] * battery_data["Max Discharge Current (A)"]

# Defining the features (capacity, maximum discharge capacity, and number of cells) and target (weight per pack)
features = battery_data[["Capacity (mAh)", "Max Discharge Current (A)", "Capacity-Discharge Interaction", "Voltage (V)"]]
target = battery_data["Weight per Pack (g)"]

# Creating and fitting the linear regression model for weight per pack
# model = LinearRegression()
model = Ridge(alpha=1.0)
model.fit(features, target)

# Extracting the coefficients and intercept of the model for weight per pack
coefficients = model.coef_
intercept = model.intercept_

# Constructing the formula to estimate battery weight per pack
formula = f"Weight per Pack (g) = {coefficients[0]:.8f} * Capacity (mAh) + {coefficients[1]:.8f} * Max Discharge Current (Ah) + {coefficients[2]:.8f} * Capacity * Discharge + {coefficients[3]:.8f} * Nominal Voltage (V) + {intercept:.2f}"

print(formula)