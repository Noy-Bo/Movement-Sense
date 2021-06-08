import xlsxwriter

# https://xlsxwriter.readthedocs.io/chart.html


def addToExcel(filename, dataX, dataX2, dataY, dataY2, heading, headingX, headingY, headingY2):
    # Workbook() takes one, non-optional, argument
    # which is the filename that we want to create.
    workbook = xlsxwriter.Workbook(filename)

    # The workbook object is then used to add new
    # worksheet via the add_worksheet() method.
    worksheet = workbook.add_worksheet()

    # Create a new Format object to formats cells
    # in worksheets using add_format() method .

    # here we create bold format object .
    bold = workbook.add_format({'bold': 1})

    # create a data list .
    headings = [headingX, headingY, "", headingX, headingY2]

    data = [
        dataX,
        dataY,
        dataX2,
        dataY2
    ]

    # Write a row of data starting from 'A1'
    # with bold format .
    worksheet.write_row('A1', headings, bold)

    # Write a column of data starting from
    # 'A2', 'B2', 'C2' respectively .
    worksheet.write_column('A2', data[0])
    worksheet.write_column('B2', data[1])

    worksheet.write_column('D2', data[2])
    worksheet.write_column('E2', data[3])

    # Create a chart object that can be added
    # to a worksheet using add_chart() method.

    # here we create a scatter chart object .
    chart1 = workbook.add_chart({'type': 'scatter'})
    chart2 = workbook.add_chart({'type': 'scatter'})

    # Add a data series to a chart
    # using add_series method.

    # Configure the first series.
    # = Sheet1 !$A$1 is equivalent to ['Sheet1', 0, 0].

    # note : spaces is not inserted in b / w
    # = and Sheet1, Sheet1 and !
    # if space is inserted it throws warning.
    #  =Sheet1!$A$1:$A$7, Sheet1!$C$1:$C$7

    chart1.add_series({
        'name': '=Sheet1!$B$1',
        'categories': '=Sheet1!$A$2:$A$' + str(len(dataX) + 1),
        'values': '=Sheet1!$B$2:$B$' + str(len(dataY) + 1),
    })

    chart2.add_series({
        'name': '=Sheet1!$E$1',
        'categories': '=Sheet1!$D$2:$D$' + str(len(dataX2) + 1),
        'values': '=Sheet1!$E$2:$E$' + str(len(dataY2) + 1),
    })

    # Configure a second series.
    # Note use of alternative syntax to define ranges.
    # [sheetname, first_row, first_col, last_row, last_col].
    # chart1.add_series({
    #     'name': ['Sheet1', 0, 2],
    #     'categories': ['Sheet1', 1, 0, 6, 0],
    #     'values': ['Sheet1', 1, 1, 6, 1],
    # })

    # Add a chart title
    chart1.set_title({'name': heading})
    chart2.set_title({'name': heading})
    # Add x-axis label
    chart1.set_x_axis({'name': headingX})
    chart2.set_x_axis({'name': headingX})
    # Add y-axis label
    chart1.set_y_axis({'name': headingY})
    chart2.set_y_axis({'name': headingY2})

    # Set an Excel chart style.
    chart1.set_style(11)
    chart2.set_style(11)

    # Set Excel chart dimensions
    chart1.set_size({'width': 1080, 'height': 360})
    chart2.set_size({'width': 1080, 'height': 360})


    # add chart to the worksheet
    # the top-left corner of a chart
    # is anchored to cell E2 .
    worksheet.insert_chart('H2', chart1)
    worksheet.insert_chart('H30', chart2)

    # Finally, close the Excel file
    # via the close() method.
    workbook.close()


def GenerateExcel(timestamps, openpose_data, vicon_data, path, filename):
    # Workbook() takes one, non-optional, argument
    # which is the filename that we want to create.
    workbook = xlsxwriter.Workbook(path + filename + '.xlsx')

    # The workbook object is then used to add new
    # worksheet via the add_worksheet() method.
    worksheet = workbook.add_worksheet()

    # Create a new Format object to formats cells
    # in worksheets using add_format() method .

    # here we create bold format object .
    bold = workbook.add_format({'bold': 1})

    # create a data list .
    headings = ['Timestamps', 'Openpose', 'Vicon']

    data = [
        timestamps,
        openpose_data,
        vicon_data
    ]

    # Write a row of data starting from 'A1'
    # with bold format .
    worksheet.write_row('A1', headings, bold)

    # Write a column of data starting from
    # 'A2', 'B2', 'C2' respectively .
    worksheet.write_column('A2', data[0])
    worksheet.write_column('B2', data[1])
    worksheet.write_column('C2', data[2])

    # Finally, close the Excel file
    # via the close() method.
    workbook.close()
