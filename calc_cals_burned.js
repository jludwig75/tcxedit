function CalculateWFM()
{
	if(document.getElementById('txtWeightWFM').value=='' || isNaN(document.getElementById('txtWeightWFM').value))
	{
		alert('Please enter a valid weight');
		document.getElementById('txtWeightWFM').focus();
		return false;
	}
	
	if(document.getElementById('txtTimeWFM').value=='' || isNaN(document.getElementById('txtTimeWFM').value))
	{
		alert('Please enter a valid time');
		document.getElementById('txtTimeWFM').focus();
		return false;
	}
	
	var hr;
	var kg;
	var cb;
	
	if(document.getElementById('selTimeWFM').value != 1)
	{
		hr = document.getElementById('txtTimeWFM').value / 60;
	}
	else
	{
		hr=document.getElementById('txtTimeWFM').value;
	}
	
	if(document.getElementById('selWeightWFM').value != 1)
	{
		kg = document.getElementById('txtWeightWFM').value * .45359237;
	}
	else
	{
		kg = document.getElementById('txtWeightWFM').value;
	}
	
	if(kg > 182 || kg < 22)
	{
		alert('Please enter a valid weight');
		document.getElementById('txtWeightWFM').focus();
		return false;
	}
	
	if(hr > 12 || hr <= 0)
	{
		alert('Please enter a valid time');
		document.getElementById('txtTimeWFM').focus();
		return false;
	}
	
	cb = Math.round((kg * document.getElementById('selPaceWFM').value) * hr);
	document.getElementById('divResultWFM').innerHTML='You burned '+ cb +' calories!';
	
}
